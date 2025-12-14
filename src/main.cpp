#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <builtin_interfaces/msg/time.h>

#include <MPU9250.h>
#include <ESP32Servo.h>
#include "sd_logger.h"

// --- Pin Configuration ---
// Ultrasonic sensors
#define TRIG_L 5
#define ECHO_L 18
#define TRIG_C 19
#define ECHO_C 23
#define TRIG_R 32
#define ECHO_R 33

// ESC control pins (Brushless motors)
#define ESC_L_PIN  25    // Left thruster ESC signal
#define ESC_R_PIN  14    // Right thruster ESC signal

// Robot physical parameters
#define THRUSTER_SEPARATION 0.15  // meters (15cm between thrusters)
#define MAX_LINEAR_VEL 0.5        // m/s (maximum forward speed)
#define MAX_ANGULAR_VEL 1.0       // rad/s (maximum turn rate)

// ESC PWM parameters (standard RC servo signal)
#define ESC_FREQ 50          // 50Hz for standard RC ESCs
#define ESC_MIN_US 1000      // Minimum pulse width (μs) - full reverse
#define ESC_NEUTRAL_US 1500  // Neutral pulse width (μs) - stopped
#define ESC_MAX_US 2000      // Maximum pulse width (μs) - full forward
#define MIN_PWM_THRESHOLD 30 // Minimum effective PWM (avoid dead band)

// Micro-ROS Configuration
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RECONNECTION_DELAY 1000  // ms between reconnection attempts
#define EXECUTOR_TIMEOUT 100     // ms timeout for executor spin

// Publishing frequency
#define PUBLISH_INTERVAL 100  // 100ms = 10Hz

// --- Hardware Objects ---
MPU9250 mpu(Wire, 0x68); // MPU9250 I2C address

// ESC servo objects (for brushless motor control)
Servo escLeft;
Servo escRight;

// SD card logger
SDLogger sdLogger;

// --- Micro-ROS Objects ---
rcl_publisher_t imu_pub;
rcl_publisher_t range_l_pub, range_c_pub, range_r_pub;
rcl_publisher_t motor_status_pub;
rcl_publisher_t connection_status_pub;
rcl_subscription_t twist_sub;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Range range_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Int32 motor_status_msg;
std_msgs__msg__Bool connection_status_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// --- State Variables ---
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

int sonar_state = 0; // 0:Left, 1:Center, 2:Right
unsigned long last_cmd_time = 0;
unsigned long last_publish_time = 0;
unsigned long last_connection_check = 0;
bool agent_connected = false;

// Motor status tracking
int left_motor_pwm = ESC_NEUTRAL_US;
int right_motor_pwm = ESC_NEUTRAL_US;

// Current sensor readings (for SD logging)
float current_sonar_left = -1.0;
float current_sonar_center = -1.0;
float current_sonar_right = -1.0;
float current_cmd_vel_linear = 0.0;
float current_cmd_vel_angular = 0.0;

// --- Error Handling ---
void error_loop() {
  Serial.println("ERROR: Micro-ROS initialization failed!");
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// --- Time Synchronization ---
// Sync time with ROS2 system via micro-ROS agent
bool sync_time() {
  // Get the current time from the agent
  unsigned long now = millis();
  return rmw_uros_epoch_synchronized();
}

void set_timestamp(builtin_interfaces__msg__Time* timestamp) {
  if (rmw_uros_epoch_synchronized()) {
    int64_t time_ns = rmw_uros_epoch_nanos();
    timestamp->sec = time_ns / 1000000000;
    timestamp->nanosec = time_ns % 1000000000;
  } else {
    // Fallback to millis-based timestamp if not synced
    unsigned long ms = millis();
    timestamp->sec = ms / 1000;
    timestamp->nanosec = (ms % 1000) * 1000000;
  }
}

// --- ESC Control Functions ---

// ESC initialization (arming sequence)
void initESCs() {
  // Attach ESCs to pins with servo PWM (50Hz, 1000-2000μs)
  escLeft.attach(ESC_L_PIN, ESC_MIN_US, ESC_MAX_US);
  escRight.attach(ESC_R_PIN, ESC_MIN_US, ESC_MAX_US);

  // Send neutral signal for ESC calibration/arming
  escLeft.writeMicroseconds(ESC_NEUTRAL_US);
  escRight.writeMicroseconds(ESC_NEUTRAL_US);

  delay(2000);  // Wait for ESCs to arm (listen for beep sequence)

  Serial.println("ESCs initialized and armed");
}

// Convert velocity (-MAX_VEL to +MAX_VEL) to ESC pulse width (1000-2000μs)
int velocityToESC(float velocity) {
  // Constrain velocity to max range
  velocity = constrain(velocity, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);

  // Map velocity to pulse width
  int pulse_us = map(
    (int)(velocity * 1000),
    (int)(-MAX_LINEAR_VEL * 1000),
    (int)(MAX_LINEAR_VEL * 1000),
    ESC_MIN_US,
    ESC_MAX_US
  );

  return pulse_us;
}

// Set thruster speed via ESC
void setThrusterSpeed(Servo& esc, float velocity, int& pwm_status) {
  int pulse_us = velocityToESC(velocity);

  // Apply minimum threshold to avoid dead band
  int delta = pulse_us - ESC_NEUTRAL_US;
  if (abs(delta) > 0 && abs(delta) < MIN_PWM_THRESHOLD) {
    pulse_us = ESC_NEUTRAL_US;
  }

  esc.writeMicroseconds(pulse_us);
  pwm_status = pulse_us;  // Store for status publishing
}

// Stop all thrusters (emergency stop)
void stopThrusters() {
  escLeft.writeMicroseconds(ESC_NEUTRAL_US);
  escRight.writeMicroseconds(ESC_NEUTRAL_US);
  left_motor_pwm = ESC_NEUTRAL_US;
  right_motor_pwm = ESC_NEUTRAL_US;
}

// --- Motor Control Callback ---
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // Update watchdog
  last_cmd_time = millis();

  // Extract velocity commands
  float linear_x = msg->linear.x;   // m/s (forward/backward)
  float angular_z = msg->angular.z; // rad/s (turn left/right)

  // Store for SD logging
  current_cmd_vel_linear = linear_x;
  current_cmd_vel_angular = angular_z;

  // Differential thrust kinematics
  float thrust_left = linear_x - (angular_z * THRUSTER_SEPARATION / 2.0);
  float thrust_right = linear_x + (angular_z * THRUSTER_SEPARATION / 2.0);

  // Apply to thrusters
  setThrusterSpeed(escLeft, thrust_left, left_motor_pwm);
  setThrusterSpeed(escRight, thrust_right, right_motor_pwm);
}

// --- Ultrasonic Measurement ---
float get_distance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // 30ms timeout (approx 5m)
  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) return -1.0;

  // Convert to meters
  float dist_m = (duration * 0.034 / 2) / 100.0;
  if (dist_m < 0.2) return -1.0; // Filter minimum distance
  return dist_m;
}

// --- Create Entities ---
bool create_entities() {
  allocator = rcl_get_default_allocator();

  // Create init options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "miro_mcu_node", "", &support));

  // Create publishers with RELIABLE QoS
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"));

  RCCHECK(rclc_publisher_init_best_effort(
    &range_l_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "ultrasonic/left"));

  RCCHECK(rclc_publisher_init_best_effort(
    &range_c_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "ultrasonic/center"));

  RCCHECK(rclc_publisher_init_best_effort(
    &range_r_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "ultrasonic/right"));

  RCCHECK(rclc_publisher_init_default(
    &motor_status_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor/status"));

  RCCHECK(rclc_publisher_init_default(
    &connection_status_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "system/connection_status"));

  // Create subscriber with RELIABLE QoS
  RCCHECK(rclc_subscription_init_default(
    &twist_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Create timer (100ms = 10Hz)
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(PUBLISH_INTERVAL),
    timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &twist_sub,
    &twist_msg,
    &subscription_callback,
    ON_NEW_DATA));

  // Initialize message frame IDs
  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.header.frame_id.capacity = strlen("imu_link") + 1;

  // Sync time with agent
  sync_time();

  return true;
}

// --- Destroy Entities ---
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_pub, &node);
  rcl_publisher_fini(&range_l_pub, &node);
  rcl_publisher_fini(&range_c_pub, &node);
  rcl_publisher_fini(&range_r_pub, &node);
  rcl_publisher_fini(&motor_status_pub, &node);
  rcl_publisher_fini(&connection_status_pub, &node);
  rcl_subscription_fini(&twist_sub, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// --- SD Card Logging Helper ---
void logCurrentData() {
  if (!sdLogger.isReady()) {
    return;
  }

  LogEntry entry;
  entry.timestamp_ms = millis();

  // IMU data
  entry.imu_orient_w = 0.0;  // Not available from MPU9250 yet (need quaternion calculation)
  entry.imu_orient_x = 0.0;
  entry.imu_orient_y = 0.0;
  entry.imu_orient_z = 0.0;
  entry.imu_accel_x = imu_msg.linear_acceleration.x;
  entry.imu_accel_y = imu_msg.linear_acceleration.y;
  entry.imu_accel_z = imu_msg.linear_acceleration.z;
  entry.imu_gyro_x = imu_msg.angular_velocity.x;
  entry.imu_gyro_y = imu_msg.angular_velocity.y;
  entry.imu_gyro_z = imu_msg.angular_velocity.z;

  // Ultrasonic data
  entry.sonar_left = current_sonar_left;
  entry.sonar_center = current_sonar_center;
  entry.sonar_right = current_sonar_right;

  // Motor command data
  entry.cmd_vel_linear_x = current_cmd_vel_linear;
  entry.cmd_vel_angular_z = current_cmd_vel_angular;
  entry.motor_left_pwm = left_motor_pwm;
  entry.motor_right_pwm = right_motor_pwm;

  // System status
  entry.ros_connected = agent_connected;

  // Add to buffer (non-blocking)
  sdLogger.log(entry);
}

// --- Timer Callback (Main Publish Logic) ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;

  if (timer == NULL || state != AGENT_CONNECTED) {
    return;
  }

  unsigned long current_time = millis();

  // Publish at 10Hz
  if (current_time - last_publish_time >= PUBLISH_INTERVAL) {
    last_publish_time = current_time;

    // 1. Publish IMU data
    if (mpu.readSensor()) {
      set_timestamp(&imu_msg.header.stamp);

      imu_msg.linear_acceleration.x = mpu.getAccelX_mss();
      imu_msg.linear_acceleration.y = mpu.getAccelY_mss();
      imu_msg.linear_acceleration.z = mpu.getAccelZ_mss();

      imu_msg.angular_velocity.x = mpu.getGyroX_rads();
      imu_msg.angular_velocity.y = mpu.getGyroY_rads();
      imu_msg.angular_velocity.z = mpu.getGyroZ_rads();

      RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
    }

    // 2. Publish ultrasonic data (round-robin)
    float dist = 0.0;
    range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    range_msg.field_of_view = 0.52; // 30 degrees
    range_msg.min_range = 0.2;
    range_msg.max_range = 4.0;

    switch (sonar_state) {
      case 0: // Left
        dist = get_distance(TRIG_L, ECHO_L);
        current_sonar_left = dist;
        range_msg.header.frame_id.data = (char*)"sonar_left_link";
        range_msg.header.frame_id.size = strlen("sonar_left_link");
        range_msg.header.frame_id.capacity = strlen("sonar_left_link") + 1;
        set_timestamp(&range_msg.header.stamp);
        range_msg.range = dist;
        RCSOFTCHECK(rcl_publish(&range_l_pub, &range_msg, NULL));
        sonar_state = 1;
        break;

      case 1: // Center
        dist = get_distance(TRIG_C, ECHO_C);
        current_sonar_center = dist;
        range_msg.header.frame_id.data = (char*)"sonar_center_link";
        range_msg.header.frame_id.size = strlen("sonar_center_link");
        range_msg.header.frame_id.capacity = strlen("sonar_center_link") + 1;
        set_timestamp(&range_msg.header.stamp);
        range_msg.range = dist;
        RCSOFTCHECK(rcl_publish(&range_c_pub, &range_msg, NULL));
        sonar_state = 2;
        break;

      case 2: // Right
        dist = get_distance(TRIG_R, ECHO_R);
        current_sonar_right = dist;
        range_msg.header.frame_id.data = (char*)"sonar_right_link";
        range_msg.header.frame_id.size = strlen("sonar_right_link");
        range_msg.header.frame_id.capacity = strlen("sonar_right_link") + 1;
        set_timestamp(&range_msg.header.stamp);
        range_msg.range = dist;
        RCSOFTCHECK(rcl_publish(&range_r_pub, &range_msg, NULL));
        sonar_state = 0;
        break;
    }

    // 3. Publish motor status (combined PWM values)
    // Encode both motors: left_pwm * 10000 + right_pwm
    motor_status_msg.data = left_motor_pwm * 10000 + right_motor_pwm;
    RCSOFTCHECK(rcl_publish(&motor_status_pub, &motor_status_msg, NULL));

    // 4. Publish connection status
    connection_status_msg.data = agent_connected;
    RCSOFTCHECK(rcl_publish(&connection_status_pub, &connection_status_msg, NULL));

    // 5. Log data to SD card at 10Hz
    logCurrentData();
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  set_microros_serial_transports(Serial);

  // Initialize pins
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  // Initialize IMU
  Wire.begin();
  delay(500);
  if (mpu.begin() < 0) {
    Serial.println("ERROR: MPU9250 initialization failed");
  } else {
    Serial.println("MPU9250 initialized successfully");
  }

  // Initialize ESCs
  Serial.println("Initializing ESCs...");
  initESCs();
  Serial.println("ESCs ready");

  // Initialize SD card logger
  Serial.println("Initializing SD card...");
  if (sdLogger.begin()) {
    if (sdLogger.startNewLog()) {
      Serial.println("SD card logging started");
    } else {
      Serial.println("WARNING: Failed to start SD logging");
    }
  } else {
    Serial.println("WARNING: SD card initialization failed - continuing without logging");
  }

  // Set initial state
  state = WAITING_AGENT;

  Serial.println("ESP32 micro-ROS node initialized");
  Serial.println("Waiting for micro-ROS agent...");
}

// --- Main Loop ---
void loop() {
  switch (state) {
    case WAITING_AGENT:
      // Check if agent is available
      if (RMW_RET_OK == rmw_uros_ping_agent(1000, 1)) {
        Serial.println("Agent detected! Creating entities...");
        state = AGENT_AVAILABLE;
      } else {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
      delay(500);
      break;

    case AGENT_AVAILABLE:
      // Create entities
      create_entities();
      Serial.println("Entities created successfully");
      Serial.println("Connected to micro-ROS agent");
      state = AGENT_CONNECTED;
      agent_connected = true;
      digitalWrite(LED_BUILTIN, HIGH);
      break;

    case AGENT_CONNECTED:
      // Normal operation - spin executor
      if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_TIMEOUT));
      } else {
        Serial.println("Agent connection lost!");
        state = AGENT_DISCONNECTED;
        agent_connected = false;
      }

      // Safety watchdog: stop motors if no cmd_vel for 1 second
      if (millis() - last_cmd_time > 1000) {
        stopThrusters();
      }

      // Update SD logger (process buffer, periodic flush)
      sdLogger.update();
      break;

    case AGENT_DISCONNECTED:
      // Cleanup and try to reconnect
      Serial.println("Attempting to reconnect...");
      stopThrusters();  // Safety first

      // Close current SD log and start new one for new session
      if (sdLogger.isReady()) {
        sdLogger.closeLog();
        delay(100);
        sdLogger.startNewLog();
      }

      destroy_entities();
      state = WAITING_AGENT;
      digitalWrite(LED_BUILTIN, LOW);
      delay(RECONNECTION_DELAY);
      break;
  }

  // Update SD logger even when not connected (for logging disconnected state)
  sdLogger.update();

  delay(10);
}
