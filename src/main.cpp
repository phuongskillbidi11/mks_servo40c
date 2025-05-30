#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"

// Định nghĩa chân UART cho ESP32 (Serial2)
#define RX_PIN 16
#define TX_PIN 17

#define MOTOR_STEPS 3200
#define MICROSTEPS 1

#define RPM_MOTOR_X 40
#define RPM_MOTOR_Y 40
#define RPM_MOTOR_Z 40

#define DIR_X 25
#define STEP_X 26
#define SLEEP_X 27
#define DIR_Y 32
#define STEP_Y 33
#define SLEEP_Y 14
#define DIR_Z 18
#define STEP_Z 19
#define SLEEP_Z 21

BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X, SLEEP_X);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y, SLEEP_Y);
BasicStepperDriver stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z, SLEEP_Z);

MultiDriver controller(stepperX, stepperY, stepperZ);

// Địa chỉ của 3 module MKS SERVO42C
const uint8_t ADDR_MOTOR_X = 0xE1; // Address for motor X
const uint8_t ADDR_MOTOR_Y = 0xE2; // Address for motor Y
const uint8_t ADDR_MOTOR_Z = 0xE3; // Address for motor Z

// Hàm tính CRC (tCHK)
uint8_t calculateCRC(uint8_t addr, uint8_t func) {
  return (addr + func) & 0xFF;
}

// Gửi lệnh UART đến MKS SERVO42C
void sendCommand(uint8_t addr, uint8_t func) {
  uint8_t crc = calculateCRC(addr, func);
  while (Serial2.available()) Serial2.read(); // Xóa buffer trước khi gửi
  Serial2.write(addr);
  Serial2.write(func);
  Serial2.write(crc);
  Serial.print("Sent to 0x");
  Serial.print(addr, HEX);
  Serial.print(": 0x");
  Serial.print(func, HEX);
  Serial.print(" 0x");
  Serial.println(crc, HEX);
  delay(150); // Tăng thời gian chờ sau khi gửi
}

// Nhận phản hồi từ MKS SERVO42C
void receiveFeedback(uint8_t addr, uint8_t command) {
  unsigned long startTime = millis();
  // Serial.print("Waiting for response from module 0x");
  // Serial.print(addr, HEX);
  // Serial.println("...");

  // Chờ tối đa 200ms để nhận đủ dữ liệu
  int expectedBytes = 0;
  if (command == 0x33) expectedBytes = 6;      // addr + 4 bytes data + crc
  else if (command == 0x30) expectedBytes = 8; // addr + 6 bytes data + crc  
  else if (command == 0x39) expectedBytes = 4; // addr + 2 bytes data + crc
  else expectedBytes = 6; // default

  while (Serial2.available() < expectedBytes && millis() - startTime < 200) {
    delay(1);
  }
  
  // Serial.print("Time elapsed: ");
  // Serial.println(millis() - startTime);
  // Serial.print("Bytes available: ");
  // Serial.println(Serial2.available());

  if (Serial2.available() >= expectedBytes) {
    // Serial.println("Enough data received, starting to read...");

    // Đọc địa chỉ nhận được
    uint8_t receivedAddr = Serial2.read();
    // Serial.print("Received address: 0x");
    // Serial.println(receivedAddr, HEX);

    // Kiểm tra địa chỉ
    if (receivedAddr != addr) {
      // Serial.print("Address mismatch! Expected: 0x");
      // Serial.print(addr, HEX);
      // Serial.print(" Received: 0x");
      // Serial.println(receivedAddr, HEX);
      while (Serial2.available()) {
        // Serial.print("Clearing buffer: 0x");
        // Serial.println(Serial2.read(), HEX);
        Serial2.read();
      }
      return;
    }
    // Serial.println("Address matched successfully.");

    // Handle data based on command
    if (command == 0x33) { // Pulse count (int32_t) - BIG ENDIAN
      uint32_t pulse = 0;
      uint8_t data[4];
      for (int i = 0; i < 4; i++) {
        data[i] = Serial2.read();
        pulse |= (uint32_t)data[i] << (24 - i * 8);
      }
      Serial2.read(); // Read CRC
      float degrees = (float)pulse * 360.0 / MOTOR_STEPS;
      Serial.print("Stepper ");
      if(addr == ADDR_MOTOR_X) Serial.print("X");
      else if (addr == ADDR_MOTOR_Y) Serial.print("Y");
      else if (addr == ADDR_MOTOR_Z) Serial.print("Z");
      Serial.print(" Pulse: ");
      Serial.print(pulse);
      Serial.print(" = ");
      Serial.print(degrees, 2);
      Serial.println("°");
    } else if (command == 0x30) { // Encoder angle (carry + value)
      int32_t carry = 0;
      uint8_t carryData[4];
      for (int i = 0; i < 4; i++) {
        carryData[i] = Serial2.read();
        carry |= (int32_t)carryData[i] << (24 - i * 8);
      }
      uint16_t value = 0;
      uint8_t valueData[2];
      for (int i = 0; i < 2; i++) {
        valueData[i] = Serial2.read();
        value |= (uint16_t)valueData[i] << (8 - i * 8);
      }
      Serial2.read(); // Read CRC
      float currentAngleInRevolution = (float)value * 360.0 / 65536.0;
      float totalAngle = (float)carry * 360.0 + currentAngleInRevolution;
      float displayAngle = fmod(totalAngle, 360.0);
      if (displayAngle < 0) displayAngle += 360.0;
      Serial.print("Stepper ");
      if(addr == ADDR_MOTOR_X) Serial.print("X");
      else if (addr == ADDR_MOTOR_Y) Serial.print("Y");
      else if (addr == ADDR_MOTOR_Z) Serial.print("Z");
      Serial.print(" Encoder: ");
      Serial.print(displayAngle, 1);
      Serial.print("° (");
      Serial.print(carry);
      Serial.println(" revs)");
    } else if (command == 0x39) { // Error angle (int16_t) - BIG ENDIAN
      uint16_t value = 0;
      uint8_t data[2];
      for (int i = 0; i < 2; i++) {
        data[i] = Serial2.read();
        value |= (uint16_t)data[i] << (8 - i * 8);
      }
      Serial2.read(); // Read CRC
      int16_t signedValue = (int16_t)value;
      float degrees = (float)signedValue * 360.0 / 65536.0;
      Serial.print("Module 0x");
      Serial.print(addr, HEX);
      Serial.print(" Error: ");
      Serial.print(degrees, 2);
      Serial.println("°");
    }
    while (Serial2.available()) Serial2.read(); // Clear buffer
  } else {
    // Serial.print("No response from module 0x");
    // Serial.println(addr, HEX);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN);

  stepperX.begin(RPM_MOTOR_X, MICROSTEPS);
  stepperX.setEnableActiveState(LOW);
  stepperX.enable();
  stepperX.setSpeedProfile(stepperX.LINEAR_SPEED, 1000, 2000);

  stepperY.begin(RPM_MOTOR_Y, MICROSTEPS);
  stepperY.setEnableActiveState(LOW);
  stepperY.enable();
  stepperY.setSpeedProfile(stepperY.LINEAR_SPEED, 1000, 2000);

  stepperZ.begin(RPM_MOTOR_Z, MICROSTEPS);
  stepperZ.setEnableActiveState(LOW);
  stepperZ.enable();
  stepperZ.setSpeedProfile(stepperZ.LINEAR_SPEED, 1000, 2000);

  Serial.println("START - 3 motors ready");

  // Hiệu chỉnh encoder cho tất cả các module
  Serial.println("Resetting encoders...");
  
  sendCommand(ADDR_MOTOR_X, 0x90); // Reset encoder module X
  delay(200);
  receiveFeedback(ADDR_MOTOR_X, 0x90);
  delay(200);

  sendCommand(ADDR_MOTOR_Y, 0x90); // Reset encoder module Y
  delay(200);
  receiveFeedback(ADDR_MOTOR_Y, 0x90);
  delay(200);

  sendCommand(ADDR_MOTOR_Z, 0x90); // Reset encoder module Z
  delay(200);
  receiveFeedback(ADDR_MOTOR_Z, 0x90);
  delay(200);

  // Đặt chế độ CR_vFOC (nếu cần)
  Serial.println("Setting CR_vFOC mode...");
  
  sendCommand(ADDR_MOTOR_X, 0x82); // Chuyển sang CR_vFOC
  delay(200);
  receiveFeedback(ADDR_MOTOR_X, 0x82);
  delay(200);

  sendCommand(ADDR_MOTOR_Y, 0x82); // Chuyển sang CR_vFOC
  delay(200);
  receiveFeedback(ADDR_MOTOR_Y, 0x82);
  delay(200);

  sendCommand(ADDR_MOTOR_Z, 0x82); // Chuyển sang CR_vFOC
  delay(200);
  receiveFeedback(ADDR_MOTOR_Z, 0x82);
  delay(200);

  Serial.println("Starting rotation...");
  controller.rotate(360, 360, 360);
}

void loop() {
  static bool allDone = false;
  static unsigned long lastTime = 0;
  static unsigned long lastFeedbackTime = 0;

  // Đọc feedback mỗi 1 giây thay vì 500ms để giảm spam
  if (millis() - lastFeedbackTime >= 1000) {
    Serial.println("\n=== READING FEEDBACK ===");
    
    // Chỉ đọc từ module X để test trước
    Serial.println("--- Module X ---");
    sendCommand(ADDR_MOTOR_X, 0x33); // Số xung
    receiveFeedback(ADDR_MOTOR_X, 0x33);
    delay(100);
    
    sendCommand(ADDR_MOTOR_X, 0x30); // Góc encoder
    receiveFeedback(ADDR_MOTOR_X, 0x30);
    delay(100);
    
    sendCommand(ADDR_MOTOR_X, 0x39); // Góc sai số
    receiveFeedback(ADDR_MOTOR_X, 0x39);
    delay(100);

    // Uncomment khi module Y và Z hoạt động
  
    Serial.println("--- Module Y ---");
    sendCommand(ADDR_MOTOR_Y, 0x33);
    receiveFeedback(ADDR_MOTOR_Y, 0x33);
    delay(100);
    sendCommand(ADDR_MOTOR_Y, 0x30);
    receiveFeedback(ADDR_MOTOR_Y, 0x30);
    delay(100);
    sendCommand(ADDR_MOTOR_Y, 0x39);
    receiveFeedback(ADDR_MOTOR_Y, 0x39);
    delay(100);

    Serial.println("--- Module Z ---");
    sendCommand(ADDR_MOTOR_Z, 0x33);
    receiveFeedback(ADDR_MOTOR_Z, 0x33);
    delay(100);
    sendCommand(ADDR_MOTOR_Z, 0x30);
    receiveFeedback(ADDR_MOTOR_Z, 0x30);
    delay(100);
    sendCommand(ADDR_MOTOR_Z, 0x39);
    receiveFeedback(ADDR_MOTOR_Z, 0x39);
    delay(100);
    

    lastFeedbackTime = millis();
  }

  unsigned wait_time = controller.nextAction();

  if (wait_time) {
    // Chỉ in thông tin motor khi có thay đổi đáng kể
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 1000) {
      if (stepperX.getStepsRemaining()) {
        Serial.print("MotorX: ");
        Serial.print(stepperX.getStepsCompleted());
        Serial.print("/");
        Serial.print(stepperX.getStepsCompleted() + stepperX.getStepsRemaining());
        Serial.print(" steps, ");
        Serial.print(stepperX.getCurrentRPM(), 1);
        Serial.println(" RPM");
      }
      lastPrintTime = millis();
    }
  } else if (!allDone) {
    stepperX.disable();
    stepperY.disable();
    stepperZ.disable();
    Serial.println("\n=== All motors stopped ===");
    allDone = true;
    lastTime = millis();
  }

  if (allDone && (millis() - lastTime > 5000)) {
    Serial.println("\n=== Restarting rotation ===");
    allDone = false;
    stepperX.enable();
    stepperY.enable();
    stepperZ.enable();
    controller.rotate(360, 360, 360);
  }
}