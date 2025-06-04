#define rotation_feedback_OK
#ifndef CONFIG_H
#define CONFIG_H

// Cấu hình UART
const int RX_PIN = 16;
const int TX_PIN = 17;
const long SERIAL_BAUD = 115200;
const long SERIAL2_BAUD = 38400;

// Cấu hình động cơ bước
const int MOTOR_STEPS = 3200;
const int MICROSTEPS = 1;

// Cấu hình thông số động cơ
struct MotorConfig {
  int dirPin;
  int stepPin;
  int sleepPin;
  uint8_t address;
  float rpm;
};

// Mảng cấu hình cho ba động cơ X, Y, Z
const MotorConfig motorConfigs[] = {
  {25, 26, 27, 0xE1, 40.0}, // Motor X
  {32, 33, 14, 0xE2, 40.0}, // Motor Y
  {18, 19, 21, 0xE3, 40.0}  // Motor Z
};

// Cấu hình thời gian
const unsigned long FEEDBACK_INTERVAL = 1000; // ms
const unsigned long COMMAND_DELAY = 100;     // ms
const unsigned long RESPONSE_TIMEOUT = 200;   // ms
const unsigned long RESTART_DELAY = 5000;    // ms
const unsigned long PRINT_INTERVAL = 1000;   // ms

// Cấu hình góc quay
const float ROTATION_ANGLE = 360.0; // Độ
const float FULL_CIRCLE = 360.0;
const float ENCODER_RESOLUTION = 65536.0;

// Cấu hình góc tùy chỉnh cho từng động cơ
struct CustomAngles {
  float angleX;
  float angleY;
  float angleZ;
  bool sequentialMode; // true = tuần tự, false = đồng thời
};

// Cài đặt mặc định - có thể thay đổi theo nhu cầu
CustomAngles customRotation = {
  90.0,   // X: 90 độ
  180.0,  // Y: 180 độ  
  270.0,  // Z: 270 độ
  false   // Chế độ đồng thời
};

// Enum cho chế độ hoạt động
enum OperationMode {
  MODE_SETUP,
  MODE_CUSTOM_ROTATION,
  MODE_FEEDBACK_ONLY
};

// Mã lệnh UART
enum Command {
  // Configuration commands
  CALIBRATE_ENCODER = 0x80,
  SET_MOTOR_TYPE = 0x81,
  SET_WORK_MODE = 0x82,
  SET_CURRENT = 0x83,
  SET_SUBDIVISION = 0x84,
  SET_EN_PIN_LEVEL = 0x85,
  SET_DIRECTION = 0x86,
  SET_AUTO_SCREEN_OFF = 0x87,
  SET_LOCKED_ROTOR_PROTECTION = 0x88,
  SET_SUBDIVISION_INTERPOLATION = 0x89,
  SET_BAUD_RATE = 0x8A,
  SET_SLAVE_ADDRESS = 0x8B,
  RESTORE_DEFAULT = 0x3F,
  
  // Zero mode commands
  SET_ZERO_MODE = 0x90,
  SET_ZERO_POINT = 0x91,
  SET_ZERO_SPEED = 0x92,
  SET_ZERO_DIRECTION = 0x93,
  GO_TO_ZERO = 0x94,
  
  // PID/ACC/Torque commands
  SET_POSITION_KP = 0xA1,
  SET_POSITION_KI = 0xA2,
  SET_POSITION_KD = 0xA3,
  SET_ACCELERATION = 0xA4,
  SET_MAX_TORQUE = 0xA5,
  
  // Read commands
  SET_CR_VFOC = 0x82,  // Same as SET_WORK_MODE
  READ_PULSE = 0x33,
  READ_ENCODER = 0x30,
  READ_ERROR = 0x39
};

// Số byte mong đợi cho từng lệnh
const int expectedBytesMap[][2] = {
  {READ_PULSE, 6},      // addr + 4 bytes data + crc
  {READ_ENCODER, 8},    // addr + 6 bytes data + crc  
  {READ_ERROR, 4},      // addr + 2 bytes data + crc
  // Configuration commands return: addr + status + crc = 3 bytes
  {CALIBRATE_ENCODER, 3},
  {SET_MOTOR_TYPE, 3},
  {SET_WORK_MODE, 3},
  {SET_CURRENT, 3},
  {SET_SUBDIVISION, 3},
  {SET_EN_PIN_LEVEL, 3},
  {SET_DIRECTION, 3},
  {SET_AUTO_SCREEN_OFF, 3},
  {SET_LOCKED_ROTOR_PROTECTION, 3},
  {SET_SUBDIVISION_INTERPOLATION, 3},
  {SET_BAUD_RATE, 3},
  {SET_SLAVE_ADDRESS, 3},
  {RESTORE_DEFAULT, 3},
  {SET_ZERO_MODE, 3},
  {SET_ZERO_POINT, 3},
  {SET_ZERO_SPEED, 3},
  {SET_ZERO_DIRECTION, 3},
  {GO_TO_ZERO, 3},
  {SET_POSITION_KP, 3},
  {SET_POSITION_KI, 3},
  {SET_POSITION_KD, 3},
  {SET_ACCELERATION, 3},
  {SET_MAX_TORQUE, 3}
};

#endif