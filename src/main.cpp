#include <Arduino.h>
#include <ESP32Servo.h>
#include <MPU9250.h>
#include <Wire.h>

// ==========================================
//        1. Pin & Robot Configuration
// ==========================================

// Ultrasonic sensors
#define TRIG_L 5
#define ECHO_L 18
#define TRIG_C 19
#define ECHO_C 23
#define TRIG_R 32
#define ECHO_R 33

// ESC control pins
#define ESC_L_PIN 25 // Left thruster
#define ESC_R_PIN 14 // Right thruster

// Robot parameters
#define THRUSTER_SEPARATION 0.15 // 15cm
#define MAX_LINEAR_VEL 0.5       // m/s
#define MAX_ANGULAR_VEL 1.0      // rad/s

// ESC Parameters (Forward Only)
#define ESC_MIN_US 1000
#define ESC_NEUTRAL_US 1000      // Stop
#define ESC_FORWARD_START_US 1484
#define ESC_MAX_US 2000

// ==========================================
//        2. Custom Protocol Definition
// ==========================================

const uint8_t HEADER[] = {0xFE, 0xFF};
const uint8_t FOOTER[] = {0x0A, 0x0D};

// Command Packet (RPi -> ESP32)
// ID: 0x01
struct __attribute__((packed)) CmdData {
    float linear_x;
    float angular_z;
};

// Telemetry Packet (ESP32 -> RPi)
// ID: 0x02
struct __attribute__((packed)) TelemetryData {
    // IMU (Acceleration & Gyro)
    float ax, ay, az;
    float gx, gy, gz;

    // Ultrasonics (Left, Center, Right)
    float dist_l;
    float dist_c;
    float dist_r;

    // Motor PWM status
    int16_t pwm_l;
    int16_t pwm_r;
};

// IMU Data Packet (ESP32 -> RPi)
// ID: 0x03
struct __attribute__((packed)) ImuData {
    float ax, ay, az;
    float gx, gy, gz;
};

// Heartbeat Packet (ESP32 -> RPi)
// ID: 0x08
struct __attribute__((packed)) HeartbeatData {
    uint8_t sequence;
};

// ==========================================
//        3. Global Objects & Variables
// ==========================================

MPU9250 mpu(Wire, 0x68);
Servo escLeft;
Servo escRight;

// Timing Constants
const unsigned long IMU_INTERVAL = 10;       // 100Hz
const unsigned long TELEM_INTERVAL = 100;    // 10Hz
const unsigned long HEARTBEAT_INTERVAL = 1000; // 1Hz

// State Variables
unsigned long last_cmd_time = 0;
unsigned long last_imu_time = 0;
unsigned long last_telem_time = 0;
unsigned long last_heartbeat_time = 0;

// Motor Status
int left_motor_pwm = ESC_NEUTRAL_US;
int right_motor_pwm = ESC_NEUTRAL_US;

// Data Buffers
CmdData cmd_packet;
TelemetryData telemetry_packet;
ImuData imu_packet;
HeartbeatData heartbeat_packet;

// Heartbeat Sequence Counter
uint8_t heartbeat_sequence = 0;

// ==========================================
//        4. Helper Functions
// ==========================================

void initESCs() {
    escLeft.attach(ESC_L_PIN, ESC_MIN_US, ESC_MAX_US);
    escRight.attach(ESC_R_PIN, ESC_MIN_US, ESC_MAX_US);
    
    // Arming sequence
    escLeft.writeMicroseconds(ESC_NEUTRAL_US);
    escRight.writeMicroseconds(ESC_NEUTRAL_US);
    delay(2000); 
}

void stopThrusters() {
    escLeft.writeMicroseconds(ESC_NEUTRAL_US);
    escRight.writeMicroseconds(ESC_NEUTRAL_US);
    left_motor_pwm = ESC_NEUTRAL_US;
    right_motor_pwm = ESC_NEUTRAL_US;
}

float get_distance(int trig, int echo) {
    digitalWrite(trig, LOW); delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    
    long duration = pulseIn(echo, HIGH, 30000); // 30ms timeout
    if (duration == 0) return -1.0;
    
    float dist_m = (duration * 0.034 / 2) / 100.0;
    if (dist_m < 0.2) return -1.0;
    return dist_m;
}

int velocityToESC(float velocity) {
    if (velocity <= 0.0) return ESC_NEUTRAL_US;
    velocity = constrain(velocity, 0.0, MAX_LINEAR_VEL);
    return map((int)(velocity * 1000), 0, (int)(MAX_LINEAR_VEL * 1000), ESC_FORWARD_START_US, ESC_MAX_US);
}

// ==========================================
//        5. Motor Logic (From Callback)
// ==========================================

void updateMotorControl(float linear_x, float angular_z) {
    last_cmd_time = millis(); // Reset Watchdog

    // Forward only logic
    if (linear_x < 0.0) linear_x = 0.0;

    // Differential Mixing
    float thrust_left = linear_x - (angular_z * THRUSTER_SEPARATION / 2.0);
    float thrust_right = linear_x + (angular_z * THRUSTER_SEPARATION / 2.0);

    thrust_left = max(0.0f, thrust_left);
    thrust_right = max(0.0f, thrust_right);

    // Apply to motors
    left_motor_pwm = velocityToESC(thrust_left);
    right_motor_pwm = velocityToESC(thrust_right);

    escLeft.writeMicroseconds(left_motor_pwm);
    escRight.writeMicroseconds(right_motor_pwm);
}

// ==========================================
//        6. Serial Communication
// ==========================================

void sendImuTelemetry() {
    // 1. Read IMU Data (fast, <1ms)
    if (mpu.readSensor()) {
        imu_packet.ax = mpu.getAccelX_mss();
        imu_packet.ay = mpu.getAccelY_mss();
        imu_packet.az = mpu.getAccelZ_mss();
        imu_packet.gx = mpu.getGyroX_rads();
        imu_packet.gy = mpu.getGyroY_rads();
        imu_packet.gz = mpu.getGyroZ_rads();
    }

    // 2. Serialize & Send
    Serial.write(HEADER, 2);
    uint8_t id = 0x03;
    Serial.write(id);
    uint8_t len = sizeof(ImuData);
    Serial.write(len);

    uint8_t* ptr = (uint8_t*)&imu_packet;
    Serial.write(ptr, len);

    // Checksum
    uint8_t checksum = 0;
    for(int i=0; i<len; i++) checksum += ptr[i];
    Serial.write(checksum);

    Serial.write(FOOTER, 2);
}

void sendTelemetry() {
    // 1. Fill IMU Data
    if (mpu.readSensor()) {
        telemetry_packet.ax = mpu.getAccelX_mss();
        telemetry_packet.ay = mpu.getAccelY_mss();
        telemetry_packet.az = mpu.getAccelZ_mss();
        telemetry_packet.gx = mpu.getGyroX_rads();
        telemetry_packet.gy = mpu.getGyroY_rads();
        telemetry_packet.gz = mpu.getGyroZ_rads();
    }

    // 2. Fill Sonar Data (Sequential Read)
    // Note: Reading all 3 sequentially might take ~60-90ms max if timeout.
    // For better performance, consider round-robin reading per loop, but 
    // for simplicity here we read all.
    telemetry_packet.dist_l = get_distance(TRIG_L, ECHO_L);
    telemetry_packet.dist_c = get_distance(TRIG_C, ECHO_C);
    telemetry_packet.dist_r = get_distance(TRIG_R, ECHO_R);

    // 3. Fill Motor Data
    telemetry_packet.pwm_l = (int16_t)left_motor_pwm;
    telemetry_packet.pwm_r = (int16_t)right_motor_pwm;

    // 4. Serialize & Send
    Serial.write(HEADER, 2);
    uint8_t id = 0x02;
    Serial.write(id);
    uint8_t len = sizeof(TelemetryData);
    Serial.write(len);

    uint8_t* ptr = (uint8_t*)&telemetry_packet;
    Serial.write(ptr, len);

    // Checksum
    uint8_t checksum = 0;
    for(int i=0; i<len; i++) checksum += ptr[i];
    Serial.write(checksum);

    Serial.write(FOOTER, 2);
}

void sendHeartbeat() {
    // 1. Increment Sequence
    heartbeat_packet.sequence = heartbeat_sequence++;

    // 2. Serialize & Send
    Serial.write(HEADER, 2);
    uint8_t id = 0x08;
    Serial.write(id);
    uint8_t len = sizeof(HeartbeatData);
    Serial.write(len);

    uint8_t* ptr = (uint8_t*)&heartbeat_packet;
    Serial.write(ptr, len);

    // Checksum
    uint8_t checksum = 0;
    for(int i=0; i<len; i++) checksum += ptr[i];
    Serial.write(checksum);

    Serial.write(FOOTER, 2);
}

void readSerial() {
    static int state = 0;
    static uint8_t buffer[32]; // Enough for CmdData
    static uint8_t idx = 0;
    static uint8_t data_len = 0;
    static uint8_t checksum = 0;

    while (Serial.available()) {
        uint8_t b = Serial.read();

        switch (state) {
            case 0: if (b == 0xFE) state = 1; break; // Header 1
            case 1: if (b == 0xFF) state = 2; else state = 0; break; // Header 2

            case 2: // ID Check
                if (b == 0x01) state = 3; // CmdData ID
                else state = 0;
                break;

            case 3: // Length Check
                data_len = b;
                if (data_len == sizeof(CmdData)) {
                    idx = 0; checksum = 0; state = 4;
                } else state = 0;
                break;

            case 4: // Payload Read
                buffer[idx++] = b;
                checksum += b;
                if (idx >= data_len) state = 5;
                break;

            case 5: // Checksum Verify
                if (b == checksum) state = 6;
                else state = 0; // Checksum fail
                break;

            case 6: // Footer 1
                if (b == 0x0A) state = 7;
                else state = 0;
                break;

            case 7: // Footer 2 (Complete)
                if (b == 0x0D) {
                    // Success! Parse struct
                    CmdData received;
                    memcpy(&received, buffer, sizeof(CmdData));
                    updateMotorControl(received.linear_x, received.angular_z);
                }
                state = 0;
                break;
        }
    }
}

// ==========================================
//        7. Main Setup & Loop
// ==========================================

void setup() {
    Serial.begin(115200);

    // Init Sensors
    pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
    pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
    pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

    Wire.begin();
    delay(500);
    mpu.begin(); // Basic init

    // Init Motors
    initESCs();
}

void loop() {
    // 1. Handle Incoming Serial Commands
    readSerial();

    // 2. Split-Rate Telemetry Scheduler
    unsigned long current_time = millis();

    // IMU: 100Hz (every 10ms)
    if (current_time - last_imu_time >= IMU_INTERVAL) {
        last_imu_time = current_time;
        sendImuTelemetry();
    }

    // Full Telemetry: 10Hz (every 100ms)
    if (current_time - last_telem_time >= TELEM_INTERVAL) {
        last_telem_time = current_time;
        sendTelemetry();
    }

    // Heartbeat: 1Hz (every 1000ms)
    if (current_time - last_heartbeat_time >= HEARTBEAT_INTERVAL) {
        last_heartbeat_time = current_time;
        sendHeartbeat();
    }

    // 3. Safety Watchdog (Stop if no command for 1s)
    if (millis() - last_cmd_time > 1000) {
        stopThrusters();
    }
}