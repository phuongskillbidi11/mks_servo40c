// #define TEST_AccelTest
/////////////////////////////////////
#define TEST_BasicStepperDriver
/////////////////////////////////////
#define control_motorstep_rpm
        ///////////////handleWebSocketMessage/////
        #define Sensor Data Dashboard 
        // #define Control MultiDriver Stepper Dashboard

#define area
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
float ROTATION_ANGLE = 360.0; 
const float FULL_CIRCLE = 360.0;
const float ENCODER_RESOLUTION = 65536.0;

// Mã lệnh UART
enum Command {
  RESET_ENCODER = 0x90,
  SET_CR_VFOC = 0x82,
  READ_PULSE = 0x33,
  READ_ENCODER = 0x30,
  READ_ERROR = 0x39
};

// Số byte mong đợi cho từng lệnh
const int expectedBytesMap[][2] = {
  {READ_PULSE, 6},
  {READ_ENCODER, 8},
  {READ_ERROR, 4}
};
