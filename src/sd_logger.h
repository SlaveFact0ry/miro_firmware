#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// --- SD Card Pin Configuration (ESP32 DevkitC default SPI) ---
#define SD_CS_PIN       15    // Chip Select
#define SD_MOSI_PIN     13    // Master Out Slave In
#define SD_MISO_PIN     12    // Master In Slave Out
#define SD_SCK_PIN      14    // Serial Clock
#define SD_STATUS_LED   2     // LED for SD card write status (GPIO2)

// --- Logging Configuration ---
#define LOG_BUFFER_SIZE 512   // Circular buffer size (number of log entries)
#define LOG_FLUSH_INTERVAL 1000  // Flush to SD every 1 second
#define LOG_FILE_PREFIX "/miro_log_"
#define LOG_FILE_EXTENSION ".csv"
#define MAX_FILENAME_LEN 64

// --- Log Data Structure ---
struct LogEntry {
  unsigned long timestamp_ms;     // Millisecond timestamp

  // IMU data
  float imu_orient_w;             // Quaternion orientation (not available yet, reserved)
  float imu_orient_x;
  float imu_orient_y;
  float imu_orient_z;
  float imu_accel_x;              // Linear acceleration (m/s²)
  float imu_accel_y;
  float imu_accel_z;
  float imu_gyro_x;               // Angular velocity (rad/s)
  float imu_gyro_y;
  float imu_gyro_z;

  // Ultrasonic data
  float sonar_left;               // Distance (m)
  float sonar_center;
  float sonar_right;

  // Motor command data
  float cmd_vel_linear_x;         // Commanded linear velocity (m/s)
  float cmd_vel_angular_z;        // Commanded angular velocity (rad/s)
  int motor_left_pwm;             // Actual PWM value (microseconds)
  int motor_right_pwm;

  // System status
  bool ros_connected;             // micro-ROS connection status
};

// --- SD Logger Class ---
class SDLogger {
private:
  File logFile;
  char currentFileName[MAX_FILENAME_LEN];
  bool isInitialized;
  bool isFileOpen;
  unsigned long lastFlushTime;
  unsigned long logCounter;

  // Circular buffer for non-blocking writes
  LogEntry logBuffer[LOG_BUFFER_SIZE];
  volatile int bufferHead;  // Write position
  volatile int bufferTail;  // Read position

  // Generate unique filename with timestamp
  void generateFileName() {
    unsigned long timestamp = millis();
    snprintf(currentFileName, MAX_FILENAME_LEN, "%s%lu%s",
             LOG_FILE_PREFIX, timestamp, LOG_FILE_EXTENSION);
  }

  // Write CSV header row
  bool writeHeader() {
    if (!isFileOpen) return false;

    logFile.println("timestamp_ms,"
                    "imu_orient_w,imu_orient_x,imu_orient_y,imu_orient_z,"
                    "imu_accel_x,imu_accel_y,imu_accel_z,"
                    "imu_gyro_x,imu_gyro_y,imu_gyro_z,"
                    "sonar_left,sonar_center,sonar_right,"
                    "cmd_vel_linear_x,cmd_vel_angular_z,"
                    "motor_left_pwm,motor_right_pwm,"
                    "ros_connected");

    return true;
  }

  // Write log entry to file
  bool writeEntry(const LogEntry& entry) {
    if (!isFileOpen) return false;

    // Format: CSV with full precision for floats
    logFile.print(entry.timestamp_ms); logFile.print(",");
    logFile.print(entry.imu_orient_w, 6); logFile.print(",");
    logFile.print(entry.imu_orient_x, 6); logFile.print(",");
    logFile.print(entry.imu_orient_y, 6); logFile.print(",");
    logFile.print(entry.imu_orient_z, 6); logFile.print(",");
    logFile.print(entry.imu_accel_x, 6); logFile.print(",");
    logFile.print(entry.imu_accel_y, 6); logFile.print(",");
    logFile.print(entry.imu_accel_z, 6); logFile.print(",");
    logFile.print(entry.imu_gyro_x, 6); logFile.print(",");
    logFile.print(entry.imu_gyro_y, 6); logFile.print(",");
    logFile.print(entry.imu_gyro_z, 6); logFile.print(",");
    logFile.print(entry.sonar_left, 3); logFile.print(",");
    logFile.print(entry.sonar_center, 3); logFile.print(",");
    logFile.print(entry.sonar_right, 3); logFile.print(",");
    logFile.print(entry.cmd_vel_linear_x, 6); logFile.print(",");
    logFile.print(entry.cmd_vel_angular_z, 6); logFile.print(",");
    logFile.print(entry.motor_left_pwm); logFile.print(",");
    logFile.print(entry.motor_right_pwm); logFile.print(",");
    logFile.println(entry.ros_connected);

    logCounter++;
    return true;
  }

public:
  SDLogger() : isInitialized(false), isFileOpen(false),
               bufferHead(0), bufferTail(0),
               lastFlushTime(0), logCounter(0) {
    memset(currentFileName, 0, MAX_FILENAME_LEN);
  }

  // Initialize SD card and SPI
  bool begin() {
    pinMode(SD_STATUS_LED, OUTPUT);
    digitalWrite(SD_STATUS_LED, LOW);

    // Initialize SPI with custom pins
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    // Initialize SD card
    if (!SD.begin(SD_CS_PIN)) {
      Serial.println("ERROR: SD card initialization failed!");
      Serial.println("Check:");
      Serial.println("  - SD card inserted?");
      Serial.println("  - Wiring correct?");
      Serial.println("  - Card formatted as FAT32?");
      isInitialized = false;
      return false;
    }

    // Verify SD card type
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("ERROR: No SD card detected");
      isInitialized = false;
      return false;
    }

    Serial.print("SD Card Type: ");
    switch(cardType) {
      case CARD_MMC:  Serial.println("MMC"); break;
      case CARD_SD:   Serial.println("SDSC"); break;
      case CARD_SDHC: Serial.println("SDHC"); break;
      default:        Serial.println("UNKNOWN"); break;
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    isInitialized = true;
    Serial.println("SD card initialized successfully");

    // Blink LED to indicate successful initialization
    for (int i = 0; i < 3; i++) {
      digitalWrite(SD_STATUS_LED, HIGH);
      delay(100);
      digitalWrite(SD_STATUS_LED, LOW);
      delay(100);
    }

    return true;
  }

  // Create new log file for session
  bool startNewLog() {
    if (!isInitialized) {
      Serial.println("ERROR: SD card not initialized");
      return false;
    }

    // Close existing file if open
    if (isFileOpen) {
      closeLog();
    }

    // Generate new filename
    generateFileName();

    // Open file for writing
    logFile = SD.open(currentFileName, FILE_WRITE);
    if (!logFile) {
      Serial.println("ERROR: Failed to create log file");
      isFileOpen = false;
      return false;
    }

    isFileOpen = true;
    logCounter = 0;

    Serial.print("Created log file: ");
    Serial.println(currentFileName);

    // Write CSV header
    if (!writeHeader()) {
      Serial.println("ERROR: Failed to write CSV header");
      closeLog();
      return false;
    }

    logFile.flush();  // Ensure header is written
    lastFlushTime = millis();

    // Reset buffer
    bufferHead = 0;
    bufferTail = 0;

    digitalWrite(SD_STATUS_LED, HIGH);

    return true;
  }

  // Add log entry to buffer (non-blocking)
  bool log(const LogEntry& entry) {
    if (!isInitialized || !isFileOpen) {
      return false;
    }

    // Calculate next head position
    int nextHead = (bufferHead + 1) % LOG_BUFFER_SIZE;

    // Check if buffer is full
    if (nextHead == bufferTail) {
      Serial.println("WARNING: Log buffer full, dropping entry");
      return false;
    }

    // Add entry to buffer
    logBuffer[bufferHead] = entry;
    bufferHead = nextHead;

    return true;
  }

  // Process buffer and write to SD (call regularly in loop)
  void update() {
    if (!isInitialized || !isFileOpen) {
      return;
    }

    unsigned long currentTime = millis();
    bool shouldFlush = false;
    int entriesWritten = 0;

    // Write buffered entries to file
    while (bufferTail != bufferHead && entriesWritten < 10) {
      // Write one entry
      if (!writeEntry(logBuffer[bufferTail])) {
        Serial.println("ERROR: Failed to write log entry");
        break;
      }

      // Move tail forward
      bufferTail = (bufferTail + 1) % LOG_BUFFER_SIZE;
      entriesWritten++;

      // Toggle LED briefly to indicate write activity
      if (entriesWritten % 5 == 0) {
        digitalWrite(SD_STATUS_LED, !digitalRead(SD_STATUS_LED));
      }
    }

    // Flush to SD periodically
    if (currentTime - lastFlushTime >= LOG_FLUSH_INTERVAL) {
      logFile.flush();
      lastFlushTime = currentTime;
      shouldFlush = true;
      digitalWrite(SD_STATUS_LED, HIGH);  // LED on when flushed
    }

    // Check for SD card errors
    if (!SD.exists(currentFileName)) {
      Serial.println("ERROR: Log file disappeared! SD card may be failing");
      isFileOpen = false;
      digitalWrite(SD_STATUS_LED, LOW);
    }
  }

  // Close current log file gracefully
  void closeLog() {
    if (isFileOpen && logFile) {
      // Write any remaining buffered entries
      while (bufferTail != bufferHead) {
        writeEntry(logBuffer[bufferTail]);
        bufferTail = (bufferTail + 1) % LOG_BUFFER_SIZE;
      }

      logFile.flush();
      logFile.close();

      Serial.print("Closed log file: ");
      Serial.println(currentFileName);
      Serial.printf("Total entries written: %lu\n", logCounter);

      isFileOpen = false;
      digitalWrite(SD_STATUS_LED, LOW);
    }
  }

  // Get current statistics
  void printStats() {
    Serial.println("=== SD Logger Status ===");
    Serial.printf("Initialized: %s\n", isInitialized ? "YES" : "NO");
    Serial.printf("File open: %s\n", isFileOpen ? "YES" : "NO");
    if (isFileOpen) {
      Serial.printf("Current file: %s\n", currentFileName);
      Serial.printf("Entries logged: %lu\n", logCounter);
      Serial.printf("Buffer usage: %d/%d\n",
                    (bufferHead - bufferTail + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE,
                    LOG_BUFFER_SIZE);
    }

    if (isInitialized) {
      uint64_t totalBytes = SD.totalBytes() / (1024 * 1024);
      uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);
      Serial.printf("SD card: %lluMB used / %lluMB total\n", usedBytes, totalBytes);
    }
    Serial.println("========================");
  }

  // Check if logger is ready
  bool isReady() {
    return isInitialized && isFileOpen;
  }

  // Get buffer fill percentage
  int getBufferFillPercent() {
    int used = (bufferHead - bufferTail + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
    return (used * 100) / LOG_BUFFER_SIZE;
  }
};

#endif // SD_LOGGER_H
