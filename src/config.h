#define rotation_feedback_OK
#ifndef CONFIG_H
// #define CONFIG_H
int currentMotorIndex = 0;
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
  true   // Chế độ đồng thời
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
// // Read Commands: Đọc dữ liệu từ các module
// // Ví dụ: Đọc số xung (pulse) từ motor 0 (X)
// sendCommand(motorConfigs[0].address, READ_PULSE);  // Note: Đọc số xung từ motor X (0xE1). Không cần tham số. Kết quả sẽ được xử lý trong uartFeedbackTask.

// // Ví dụ: Đọc giá trị encoder từ motor 1 (Y)
// sendCommand(motorConfigs[1].address, READ_ENCODER);  // Note: Đọc giá trị encoder (số vòng quay và góc) từ motor Y (0xE2). Kết quả sẽ được in qua serialPrintTask.

// // Ví dụ: Đọc sai số (error) từ motor 2 (Z)
// sendCommand(motorConfigs[2].address, READ_ERROR);  // Note: Đọc sai số của encoder từ motor Z (0xE3). Kết quả hiển thị độ lệch (độ).

// // Set Parameters Commands: Cấu hình tham số cho động cơ
// // Ví dụ: Thiết lập current cho motor 0 (X)
// sendCommand(motorConfigs[0].address, SET_CURRENT, MotorDefaults::CURRENT_DEFAULT);  // Note: Đặt dòng điện cho motor X ở mức mặc định (0x08). Giá trị từ 0x00 đến 0x0F.

// // Ví dụ: Thiết lập subdivision cho motor 1 (Y)
// sendCommand(motorConfigs[1].address, SET_SUBDIVISION, MotorDefaults::MICROSTEP_16);  // Note: Đặt 16 microsteps cho motor Y (0x04). Phạm vi từ 0x00 (1 microstep) đến 0x05 (32 microsteps).

// // Ví dụ: Calibrate encoder cho motor 2 (Z)
// sendCommand(motorConfigs[2].address, CALIBRATE_ENCODER, 0x00);  // Note: Khởi động calibrate encoder cho motor Z. Tham số 0x00 là mặc định.

// // Ví dụ: Đặt loại động cơ cho motor 0 (X)
// sendCommand(motorConfigs[0].address, SET_MOTOR_TYPE, MotorDefaults::MOTOR_TYPE_STEPPER);  // Note: Đặt motor X là loại stepper (0x00). Có thể chọn MOTOR_TYPE_SERVO (0x01).

// // Ví dụ: Đặt chế độ làm việc cho motor 1 (Y)
// sendCommand(motorConfigs[1].address, SET_WORK_MODE, MotorDefaults::MODE_CLOSED_LOOP);  // Note: Đặt motor Y ở chế độ vòng khép kín (0x01). Có thể chọn MODE_OPEN_LOOP (0x00) hoặc MODE_CR_VFOC (0x02).

// // Ví dụ: Đặt mức hoạt động chân enable cho motor 2 (Z)
// sendCommand(motorConfigs[2].address, SET_EN_PIN_LEVEL, MotorDefaults::EN_ACTIVE_LOW);  // Note: Đặt chân enable của motor Z ở mức thấp (0x00). Có thể chọn EN_ACTIVE_HIGH (0x01) hoặc EN_ALWAYS_ON (0x02).

// // Ví dụ: Đặt hướng quay cho motor 0 (X)
// sendCommand(motorConfigs[0].address, SET_DIRECTION, MotorDefaults::DIR_REVERSE);  // Note: Đặt hướng quay ngược cho motor X (0x01). Mặc định là DIR_NORMAL (0x00).

// // Ví dụ: Bật bảo vệ rotor bị khóa cho motor 1 (Y)
// sendCommand(motorConfigs[1].address, SET_LOCKED_ROTOR_PROTECTION, 0x01);  // Note: Bật bảo vệ rotor bị khóa cho motor Y (0x01). 0x00 để tắt.

// // Ví dụ: Đặt baud rate cho motor 2 (Z)
// sendCommand(motorConfigs[2].address, SET_BAUD_RATE, MotorDefaults::BAUD_115200);  // Note: Đặt baud rate của motor Z là 115200 (0x05). Sau lệnh này, cần cập nhật Serial2.begin(115200, ...).

// // Ví dụ: Đặt địa chỉ slave mới cho motor 0 (X)
// sendCommand(motorConfigs[0].address, SET_SLAVE_ADDRESS, 0xE4);  // Note: Đổi địa chỉ của motor X thành 0xE4. Cần cập nhật motorConfigs[0].address sau khi thành công.

// // Ví dụ: Khôi phục cài đặt mặc định cho motor 1 (Y)
// sendCommand(motorConfigs[1].address, RESTORE_DEFAULT, 0x00);  // Note: Khôi phục cài đặt gốc cho motor Y. Tham số 0x00 là mặc định.

// // Zero Mode Commands: Cài đặt chế độ điểm gốc
// // Ví dụ: Đặt chế độ điểm gốc cho motor 0 (X)
// sendCommand(motorConfigs[0].address, SET_ZERO_MODE, MotorDefaults::ZERO_MODE_ENABLED);  // Note: Bật chế độ điểm gốc cho motor X (0x01). Có thể chọn ZERO_MODE_DISABLED (0x00) hoặc ZERO_MODE_AUTO (0x02).

// // Ví dụ: Đặt điểm gốc cho motor 1 (Y)
// sendCommand(motorConfigs[1].address, SET_ZERO_POINT, 0x00);  // Note: Đặt điểm gốc hiện tại cho motor Y. Tham số 0x00 là mặc định.

// // Ví dụ: Đặt tốc độ quay về điểm gốc cho motor 2 (Z)
// sendCommand(motorConfigs[2].address, SET_ZERO_SPEED, MotorDefaults::ZERO_SPEED_MAX);  // Note: Đặt tốc độ tối đa khi quay về điểm gốc cho motor Z (0x04). Phạm vi từ 0x00 đến 0x04.

// // Ví dụ: Đặt hướng quay về điểm gốc cho motor 0 (X)
// sendCommand(motorConfigs[0].address, SET_ZERO_DIRECTION, MotorDefaults::DIR_NORMAL);  // Note: Đặt hướng quay về điểm gốc là thuận cho motor X (0x00). Có thể chọn DIR_REVERSE (0x01).

// // Ví dụ: Quay về điểm gốc cho motor 1 (Y)
// sendCommand(motorConfigs[1].address, GO_TO_ZERO, 0x00);  // Note: Ra lệnh cho motor Y quay về điểm gốc. Tham số 0x00 là mặc định.

// // PID/ACC/Torque Commands: Cài đặt PID, gia tốc, mô-men xoắn
// // Ví dụ: Đặt hằng số KP cho PID của motor 0 (X)
// sendCommand(motorConfigs[0].address, SET_POSITION_KP, 0x05);  // Note: Đặt hằng số tỷ lệ (KP) cho PID của motor X. Giá trị 0x05 cần kiểm tra tài liệu.

// // Ví dụ: Đặt hằng số KI cho PID của motor 1 (Y)
// sendCommand(motorConfigs[1].address, SET_POSITION_KI, 0x02);  // Note: Đặt hằng số tích phân (KI) cho PID của motor Y. Giá trị 0x02 cần kiểm tra tài liệu.

// // Ví dụ: Đặt hằng số KD cho PID của motor 2 (Z)
// sendCommand(motorConfigs[2].address, SET_POSITION_KD, 0x01);  // Note: Đặt hằng số vi phân (KD) cho PID của motor Z. Giá trị 0x01 cần kiểm tra tài liệu.

// // Ví dụ: Đặt gia tốc cho motor 0 (X)
// sendCommand(motorConfigs[0].address, SET_ACCELERATION, 0x03);  // Note: Đặt gia tốc cho motor X. Giá trị 0x03 cần kiểm tra tài liệu.

// // Ví dụ: Đặt mô-men xoắn tối đa cho motor 1 (Y)
// sendCommand(motorConfigs[1].address, SET_MAX_TORQUE, 0x0F);  // Note: Đặt mô-men xoắn tối đa cho motor Y. Giá trị 0x0F cần kiểm tra tài liệu.

// // Legacy Aliases: Lệnh tương đương cũ
// // Ví dụ: Reset encoder cho motor 2 (Z)
// sendCommand(motorConfigs[2].address, RESET_ENCODER, MotorDefaults::ZERO_MODE_ENABLED);  // Note: Reset encoder cho motor Z (tương đương SET_ZERO_MODE). Tham số 0x01 để bật chế độ điểm gốc.

// // Ví dụ: Đặt chế độ CR_vFOC cho motor 0 (X)
// sendCommand(motorConfigs[0].address, SET_CR_VFOC, MotorDefaults::MODE_CR_VFOC);  // Note: Đặt chế độ CR_vFOC cho motor X (tương đương SET_WORK_MODE với MODE_CR_VFOC = 0x02).