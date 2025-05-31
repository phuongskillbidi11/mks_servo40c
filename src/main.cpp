#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "config.h"

// Khởi tạo động cơ bước
BasicStepperDriver steppers[] = {
  BasicStepperDriver(MOTOR_STEPS, motorConfigs[0].dirPin, motorConfigs[0].stepPin, motorConfigs[0].sleepPin),
  BasicStepperDriver(MOTOR_STEPS, motorConfigs[1].dirPin, motorConfigs[1].stepPin, motorConfigs[1].sleepPin),
  BasicStepperDriver(MOTOR_STEPS, motorConfigs[2].dirPin, motorConfigs[2].stepPin, motorConfigs[2].sleepPin)
};

MultiDriver controller(steppers[0], steppers[1], steppers[2]);

// Hàm tính CRC
uint8_t calculateCRC(uint8_t addr, uint8_t func) {
  return (addr + func) & 0xFF;
}

// Gửi lệnh UART
void sendCommand(uint8_t addr, uint8_t func) {
  uint8_t crc = calculateCRC(addr, func);
  while (Serial2.available()) Serial2.read();
  Serial2.write(addr);
  Serial2.write(func);
  Serial2.write(crc);
  Serial.print("Sent to 0x");
  Serial.print(addr, HEX);
  Serial.print(": 0x");
  Serial.print(func, HEX);
  Serial.print(" 0x");
  Serial.println(crc, HEX);
  delay(COMMAND_DELAY);
}

// Nhận phản hồi từ module
void receiveFeedback(uint8_t addr, uint8_t command) {
  unsigned long startTime = millis();
  int expectedBytes = 6; // Giá trị mặc định
  for (int i = 0; i < sizeof(expectedBytesMap) / sizeof(expectedBytesMap[0]); i++) {
    if (expectedBytesMap[i][0] == command) {
      expectedBytes = expectedBytesMap[i][1];
      break;
    }
  }

  while (Serial2.available() < expectedBytes && millis() - startTime < RESPONSE_TIMEOUT) {
    delay(1);
  }

  if (Serial2.available() >= expectedBytes) {
    uint8_t receivedAddr = Serial2.read();
    if (receivedAddr != addr) {
      while (Serial2.available()) Serial2.read();
      return;
    }

    if (command == READ_PULSE) {
      uint32_t pulse = 0;
      for (int i = 0; i < 4; i++) {
        pulse |= (uint32_t)Serial2.read() << (24 - i * 8);
      }
      Serial2.read(); // CRC
      float degrees = (float)pulse * FULL_CIRCLE / MOTOR_STEPS;
      Serial.print("Stepper ");
      Serial.print(addr == motorConfigs[0].address ? "X" : addr == motorConfigs[1].address ? "Y" : "Z");
      Serial.print(" Pulse: ");
      Serial.print(pulse);
      Serial.print(" = ");
      Serial.print(degrees, 2);
      Serial.println("°");
    } else if (command == READ_ENCODER) {
      int32_t carry = 0;
      for (int i = 0; i < 4; i++) {
        carry |= (int32_t)Serial2.read() << (24 - i * 8);
      }
      uint16_t value = 0;
      for (int i = 0; i < 2; i++) {
        value |= (uint16_t)Serial2.read() << (8 - i * 8);
      }
      Serial2.read(); // CRC
      float currentAngleInRevolution = (float)value * FULL_CIRCLE / ENCODER_RESOLUTION;
      float totalAngle = (float)carry * FULL_CIRCLE + currentAngleInRevolution;
      float displayAngle = fmod(totalAngle, FULL_CIRCLE);
      if (displayAngle < 0) displayAngle += FULL_CIRCLE;
      Serial.print("Stepper ");
      Serial.print(addr == motorConfigs[0].address ? "X" : addr == motorConfigs[1].address ? "Y" : "Z");
      Serial.print(" Encoder: ");
      Serial.print(displayAngle, 1);
      Serial.print("° (");
      Serial.print(carry);
      Serial.println(" revs)");
    } else if (command == READ_ERROR) {
      uint16_t value = 0;
      for (int i = 0; i < 2; i++) {
        value |= (uint16_t)Serial2.read() << (8 - i * 8);
      }
      Serial2.read(); // CRC
      int16_t signedValue = (int16_t)value;
      float degrees = (float)signedValue * FULL_CIRCLE / ENCODER_RESOLUTION;
      Serial.print("Stepper ");
      Serial.print(addr == motorConfigs[0].address ? "X" : addr == motorConfigs[1].address ? "Y" : "Z");
      Serial.print(" Error: ");
      Serial.print(degrees, 2);
      Serial.println("°");
    }
    while (Serial2.available()) Serial2.read();
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  for (int i = 0; i < 3; i++) {
    steppers[i].begin(motorConfigs[i].rpm, MICROSTEPS);
    steppers[i].setEnableActiveState(LOW);
    steppers[i].enable();
    steppers[i].setSpeedProfile(steppers[i].LINEAR_SPEED, 1000, 2000);
  }

  Serial.println("START - 3 motors ready");
  Serial.println("Resetting encoders...");
  for (int i = 0; i < 3; i++) {
    sendCommand(motorConfigs[i].address, RESET_ENCODER);
    receiveFeedback(motorConfigs[i].address, RESET_ENCODER);
    delay(COMMAND_DELAY);
  }

  Serial.println("Setting CR_vFOC mode...");
  for (int i = 0; i < 3; i++) {
    sendCommand(motorConfigs[i].address, SET_CR_VFOC);
    receiveFeedback(motorConfigs[i].address, SET_CR_VFOC);
    delay(COMMAND_DELAY);
  }

  Serial.println("Starting rotation...");
  controller.rotate(ROTATION_ANGLE, ROTATION_ANGLE, ROTATION_ANGLE);
}

void loop() {
  static bool allDone = false;
  static unsigned long lastTime = 0;
  static unsigned long lastFeedbackTime = 0;

  if (millis() - lastFeedbackTime >= FEEDBACK_INTERVAL) {
    Serial.println("\n=== READING FEEDBACK ===");
    for (int i = 0; i < 3; i++) {
      Serial.print("--- Module ");
      Serial.print(i == 0 ? "X" : i == 1 ? "Y" : "Z");
      Serial.println(" ---");
      sendCommand(motorConfigs[i].address, READ_PULSE);
      receiveFeedback(motorConfigs[i].address, READ_PULSE);
      delay(COMMAND_DELAY);
      sendCommand(motorConfigs[i].address, READ_ENCODER);
      receiveFeedback(motorConfigs[i].address, READ_ENCODER);
      delay(COMMAND_DELAY);
      sendCommand(motorConfigs[i].address, READ_ERROR);
      receiveFeedback(motorConfigs[i].address, READ_ERROR);
      delay(COMMAND_DELAY);
    }
    lastFeedbackTime = millis();
  }

  unsigned wait_time = controller.nextAction();
  if (wait_time) {
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= PRINT_INTERVAL) {
      for (int i = 0; i < 3; i++) {
        if (steppers[i].getStepsRemaining()) {
          Serial.print("Motor ");
          Serial.print(i == 0 ? "X" : i == 1 ? "Y" : "Z");
          Serial.print(": ");
          Serial.print(steppers[i].getStepsCompleted());
          Serial.print("/");
          Serial.print(steppers[i].getStepsCompleted() + steppers[i].getStepsRemaining());
          Serial.print(" steps, ");
          Serial.print(steppers[i].getCurrentRPM(), 1);
          Serial.println(" RPM");
        }
      }
      lastPrintTime = millis();
    }
  } else if (!allDone) {
    for (int i = 0; i < 3; i++) {
      steppers[i].disable();
    }
    Serial.println("\n=== All motors stopped ===");
    allDone = true;
    lastTime = millis();
  }

  if (allDone && (millis() - lastTime > RESTART_DELAY)) {
    Serial.println("\n=== Restarting rotation ===");
    allDone = false;
    for (int i = 0; i < 3; i++) {
      steppers[i].enable();
    }
    controller.rotate(ROTATION_ANGLE, ROTATION_ANGLE, ROTATION_ANGLE);
  }
}