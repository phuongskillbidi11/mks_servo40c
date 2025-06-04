#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "config.h"
#ifndef rotation_feedback_OK
// FreeRTOS handles
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t uartFeedbackTaskHandle = NULL;
TaskHandle_t serialPrintTaskHandle = NULL;
QueueHandle_t feedbackQueue = NULL;
SemaphoreHandle_t serial2Mutex = NULL;
SemaphoreHandle_t serialMutex = NULL;
TimerHandle_t restartTimer = NULL;

// Global variables
BasicStepperDriver steppers[] = {
  BasicStepperDriver(MOTOR_STEPS, motorConfigs[0].dirPin, motorConfigs[0].stepPin, motorConfigs[0].sleepPin),
  BasicStepperDriver(MOTOR_STEPS, motorConfigs[1].dirPin, motorConfigs[1].stepPin, motorConfigs[1].sleepPin),
  BasicStepperDriver(MOTOR_STEPS, motorConfigs[2].dirPin, motorConfigs[2].stepPin, motorConfigs[2].sleepPin)
};

MultiDriver controller(steppers[0], steppers[1], steppers[2]);

// Struct for feedback data
struct FeedbackData {
  uint8_t motorIndex;
  uint8_t command;
  bool success;
  union {
    struct {
      uint32_t pulse;
      float degrees;
    } pulseData;
    struct {
      int32_t carry;
      uint16_t value;
      float totalAngle;
      float displayAngle;
    } encoderData;
    struct {
      int16_t error;
      float degrees;
    } errorData;
  };
};

// Utility functions
uint8_t calculateCRC(uint8_t addr, uint8_t func) {
  return (addr + func) & 0xFF;
}

uint8_t calculateDataCRC(uint8_t addr, uint8_t* data, int dataLen) {
  uint8_t crc = addr;
  for (int i = 0; i < dataLen; i++) {
    crc += data[i];
  }
  return crc & 0xFF;
}

// Thread-safe UART communication
bool sendCommand(uint8_t addr, uint8_t func) {
  if (xSemaphoreTake(serial2Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    uint8_t crc = calculateCRC(addr, func);
    
    // Clear buffer
    while (Serial2.available()) {
      Serial2.read();
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    Serial2.write(addr);
    Serial2.write(func);
    Serial2.write(crc);
    
    xSemaphoreGive(serial2Mutex);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
    return true;
  }
  return false;
}

// Thread-safe feedback receiving
bool receiveFeedback(uint8_t addr, uint8_t command, FeedbackData* feedback) {
  if (xSemaphoreTake(serial2Mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }
  
  bool success = false;
  TickType_t startTime = xTaskGetTickCount();
  int expectedBytes = 6; // Default
  
  // Find expected bytes for command
  for (int i = 0; i < sizeof(expectedBytesMap) / sizeof(expectedBytesMap[0]); i++) {
    if (expectedBytesMap[i][0] == command) {
      expectedBytes = expectedBytesMap[i][1];
      break;
    }
  }

  // Wait for data with timeout
  while (Serial2.available() < expectedBytes && 
         (xTaskGetTickCount() - startTime) < pdMS_TO_TICKS(RESPONSE_TIMEOUT)) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (Serial2.available() >= expectedBytes) {
    uint8_t receivedAddr = Serial2.read();
    if (receivedAddr == addr) {
      feedback->success = true;
      
      if (command == READ_PULSE) {
        uint8_t data[4];
        for (int i = 0; i < 4; i++) {
          data[i] = Serial2.read();
        }
        uint8_t crc = Serial2.read();
        uint8_t calculatedCRC = calculateDataCRC(receivedAddr, data, 4);
        
        if (crc == calculatedCRC) {
          uint32_t pulse = 0;
          for (int i = 0; i < 4; i++) {
            pulse |= (uint32_t)data[i] << (24 - i * 8);
          }
          feedback->pulseData.pulse = pulse;
          feedback->pulseData.degrees = (float)pulse * FULL_CIRCLE / MOTOR_STEPS;
          success = true;
        }
      } 
      else if (command == READ_ENCODER) {
        uint8_t carryData[4], valueData[2];
        for (int i = 0; i < 4; i++) carryData[i] = Serial2.read();
        for (int i = 0; i < 2; i++) valueData[i] = Serial2.read();
        uint8_t crc = Serial2.read();
        
        uint8_t allData[6];
        memcpy(allData, carryData, 4);
        memcpy(allData + 4, valueData, 2);
        uint8_t calculatedCRC = calculateDataCRC(receivedAddr, allData, 6);
        
        if (crc == calculatedCRC) {
          int32_t carry = 0;
          for (int i = 0; i < 4; i++) {
            carry |= (int32_t)carryData[i] << (24 - i * 8);
          }
          uint16_t value = 0;
          for (int i = 0; i < 2; i++) {
            value |= (uint16_t)valueData[i] << (8 - i * 8);
          }
          
          feedback->encoderData.carry = carry;
          feedback->encoderData.value = value;
          float currentAngleInRevolution = (float)value * FULL_CIRCLE / ENCODER_RESOLUTION;
          feedback->encoderData.totalAngle = (float)carry * FULL_CIRCLE + currentAngleInRevolution;
          feedback->encoderData.displayAngle = fmod(feedback->encoderData.totalAngle, FULL_CIRCLE);
          if (feedback->encoderData.displayAngle < 0) 
            feedback->encoderData.displayAngle += FULL_CIRCLE;
          success = true;
        }
      }
      else if (command == READ_ERROR) {
        uint8_t data[2];
        for (int i = 0; i < 2; i++) data[i] = Serial2.read();
        uint8_t crc = Serial2.read();
        uint8_t calculatedCRC = calculateDataCRC(receivedAddr, data, 2);
        
        if (crc == calculatedCRC) {
          uint16_t value = 0;
          for (int i = 0; i < 2; i++) {
            value |= (uint16_t)data[i] << (8 - i * 8);
          }
          feedback->errorData.error = (int16_t)value;
          feedback->errorData.degrees = (float)feedback->errorData.error * FULL_CIRCLE / ENCODER_RESOLUTION;
          success = true;
        }
      }
    }
    
    // Clear remaining data
    while (Serial2.available()) Serial2.read();
  }
  
  feedback->success = success;
  xSemaphoreGive(serial2Mutex);
  return success;
}

// Thread-safe serial print
void safePrint(const char* message) {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.print(message);
    xSemaphoreGive(serialMutex);
  }
}

void safePrintln(const char* message) {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println(message);
    xSemaphoreGive(serialMutex);
  }
}

// FreeRTOS Tasks
void motorControlTask(void* parameter) {
  TickType_t lastPrintTime = 0;
  bool allDone = false;
  
  while (true) {
    unsigned wait_time = controller.nextAction();
    
    if (wait_time) {
      // Motors are running
      if ((xTaskGetTickCount() - lastPrintTime) >= pdMS_TO_TICKS(PRINT_INTERVAL)) {
        for (int i = 0; i < 3; i++) {
          if (steppers[i].getStepsRemaining()) {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
              Serial.print("Motor ");
              Serial.print(i == 0 ? "X" : i == 1 ? "Y" : "Z");
              Serial.print(": ");
              Serial.print(steppers[i].getStepsCompleted());
              Serial.print("/");
              Serial.print(steppers[i].getStepsCompleted() + steppers[i].getStepsRemaining());
              Serial.print(" steps, ");
              Serial.print(steppers[i].getCurrentRPM(), 1);
              Serial.println(" RPM");
              xSemaphoreGive(serialMutex);
            }
          }
        }
        lastPrintTime = xTaskGetTickCount();
      }
      allDone = false;
    } 
    else if (!allDone) {
      // Motors finished
      for (int i = 0; i < 3; i++) {
        steppers[i].disable();
      }
      safePrintln("\n=== All motors stopped ===");
      allDone = true;
      
      // Start restart timer
      xTimerStart(restartTimer, 0);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void uartFeedbackTask(void* parameter) {
  FeedbackData feedback;
  TickType_t lastFeedbackTime = 0;
  
  while (true) {
    if ((xTaskGetTickCount() - lastFeedbackTime) >= pdMS_TO_TICKS(FEEDBACK_INTERVAL)) {
      safePrintln("\n=== READING FEEDBACK ===");
      
      for (int i = 0; i < 3; i++) {
        feedback.motorIndex = i;
        
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          Serial.print("--- Module ");
          Serial.print(i == 0 ? "X" : i == 1 ? "Y" : "Z");
          Serial.println(" ---");
          xSemaphoreGive(serialMutex);
        }
        
        // Read pulse
        feedback.command = READ_PULSE;
        if (sendCommand(motorConfigs[i].address, READ_PULSE)) {
          if (receiveFeedback(motorConfigs[i].address, READ_PULSE, &feedback)) {
            xQueueSend(feedbackQueue, &feedback, 0);
          }
        }
        
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
        
        // Read encoder
        feedback.command = READ_ENCODER;
        if (sendCommand(motorConfigs[i].address, READ_ENCODER)) {
          if (receiveFeedback(motorConfigs[i].address, READ_ENCODER, &feedback)) {
            xQueueSend(feedbackQueue, &feedback, 0);
          }
        }
        
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
        
        // Read error
        feedback.command = READ_ERROR;
        if (sendCommand(motorConfigs[i].address, READ_ERROR)) {
          if (receiveFeedback(motorConfigs[i].address, READ_ERROR, &feedback)) {
            xQueueSend(feedbackQueue, &feedback, 0);
          }
        }
        
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
      }
      
      lastFeedbackTime = xTaskGetTickCount();
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void serialPrintTask(void* parameter) {
  FeedbackData feedback;
  
  while (true) {
    if (xQueueReceive(feedbackQueue, &feedback, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        char motorName = feedback.motorIndex == 0 ? 'X' : 
                        feedback.motorIndex == 1 ? 'Y' : 'Z';
        
        if (feedback.command == READ_PULSE && feedback.success) {
          Serial.print("Stepper ");
          Serial.print(motorName);
          Serial.print(" Pulse: ");
          Serial.print(feedback.pulseData.pulse);
          Serial.print(" = ");
          Serial.print(feedback.pulseData.degrees, 2);
          Serial.println("°");
        }
        else if (feedback.command == READ_ENCODER && feedback.success) {
          Serial.print("Stepper ");
          Serial.print(motorName);
          Serial.print(" Encoder: ");
          Serial.print(feedback.encoderData.displayAngle, 1);
          Serial.print("° (");
          Serial.print(feedback.encoderData.carry);
          Serial.println(" revs)");
        }
        else if (feedback.command == READ_ERROR && feedback.success) {
          Serial.print("Stepper ");
          Serial.print(motorName);
          Serial.print(" Error: ");
          Serial.print(feedback.errorData.degrees, 2);
          Serial.println("°");
        }
        
        xSemaphoreGive(serialMutex);
      }
    }
  }
}

// Timer callback for restart
void restartTimerCallback(TimerHandle_t xTimer) {
  safePrintln("\n=== Restarting rotation ===");
  
  for (int i = 0; i < 3; i++) {
    steppers[i].enable();
  }
  controller.rotate(ROTATION_ANGLE, ROTATION_ANGLE, ROTATION_ANGLE);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  // Initialize steppers
  for (int i = 0; i < 3; i++) {
    steppers[i].begin(motorConfigs[i].rpm, MICROSTEPS);
    steppers[i].setEnableActiveState(LOW);
    steppers[i].enable();
    steppers[i].setSpeedProfile(steppers[i].LINEAR_SPEED, 1000, 2000);
  }

  // Create FreeRTOS objects
  serial2Mutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  feedbackQueue = xQueueCreate(10, sizeof(FeedbackData));
  restartTimer = xTimerCreate("RestartTimer", pdMS_TO_TICKS(RESTART_DELAY), 
                              pdFALSE, (void*)0, restartTimerCallback);

  if (serial2Mutex == NULL || serialMutex == NULL || 
      feedbackQueue == NULL || restartTimer == NULL) {
    Serial.println("Failed to create FreeRTOS objects!");
    while(1);
  }

  Serial.println("START - 3 motors ready");
  Serial.println("Resetting encoders...");
  
  // Reset encoders
  for (int i = 0; i < 3; i++) {
    sendCommand(motorConfigs[i].address, RESET_ENCODER);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
  }

  Serial.println("Setting CR_vFOC mode...");
  for (int i = 0; i < 3; i++) {
    sendCommand(motorConfigs[i].address, SET_CR_VFOC);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
  }

  Serial.println("Starting rotation...");
  controller.rotate(ROTATION_ANGLE, ROTATION_ANGLE, ROTATION_ANGLE);

  // Create tasks
  xTaskCreatePinnedToCore(motorControlTask, "MotorControl", 4096, NULL, 2, 
                         &motorControlTaskHandle, 1);
  xTaskCreatePinnedToCore(uartFeedbackTask, "UARTFeedback", 4096, NULL, 2, 
                         &uartFeedbackTaskHandle, 0);
  xTaskCreatePinnedToCore(serialPrintTask, "SerialPrint", 2048, NULL, 1, 
                         &serialPrintTaskHandle, 0);

  Serial.println("FreeRTOS tasks started!");
}

void loop() {
  // Empty - all work done by FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

#endif // rotation_feedback_OK



// FreeRTOS handles
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t uartFeedbackTaskHandle = NULL;
TaskHandle_t serialPrintTaskHandle = NULL;
QueueHandle_t feedbackQueue = NULL;
SemaphoreHandle_t serial2Mutex = NULL;
SemaphoreHandle_t serialMutex = NULL;
TimerHandle_t restartTimer = NULL;

// Global variables
BasicStepperDriver steppers[] = {
  BasicStepperDriver(MOTOR_STEPS, motorConfigs[0].dirPin, motorConfigs[0].stepPin, motorConfigs[0].sleepPin),
  BasicStepperDriver(MOTOR_STEPS, motorConfigs[1].dirPin, motorConfigs[1].stepPin, motorConfigs[1].sleepPin),
  BasicStepperDriver(MOTOR_STEPS, motorConfigs[2].dirPin, motorConfigs[2].stepPin, motorConfigs[2].sleepPin)
};

MultiDriver controller(steppers[0], steppers[1], steppers[2]);

// Struct for feedback data
struct FeedbackData {
  uint8_t motorIndex;
  uint8_t command;
  bool success;
  union {
    struct {
      uint32_t pulse;
      float degrees;
    } pulseData;
    struct {
      int32_t carry;
      uint16_t value;
      float totalAngle;
      float displayAngle;
    } encoderData;
    struct {
      int16_t error;
      float degrees;
    } errorData;
    struct {
      uint8_t status;
    } statusData;
  };
};

// Utility functions
uint8_t calculateCRC(uint8_t addr, uint8_t func) {
  return (addr + func) & 0xFF;
}

uint8_t calculateDataCRC(uint8_t addr, uint8_t* data, int dataLen) {
  uint8_t crc = addr;
  for (int i = 0; i < dataLen; i++) {
    crc += data[i];
  }
  return crc & 0xFF;
}

uint8_t calculateCRCWithParam(uint8_t addr, uint8_t func, uint8_t param) {
  return (addr + func + param) & 0xFF;
}

// Thread-safe UART communication for commands without parameters
bool sendCommand(uint8_t addr, uint8_t func) {
  if (xSemaphoreTake(serial2Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    uint8_t crc = calculateCRC(addr, func);
    
    // Clear buffer
    while (Serial2.available()) {
      Serial2.read();
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    Serial2.write(addr);
    Serial2.write(func);
    Serial2.write(crc);
    
    xSemaphoreGive(serial2Mutex);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
    return true;
  }
  return false;
}

// Thread-safe UART communication for commands with parameters
bool sendCommandWithParam(uint8_t addr, uint8_t func, uint8_t param) {
  if (xSemaphoreTake(serial2Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    uint8_t crc = calculateCRCWithParam(addr, func, param);
    
    // Clear buffer
    while (Serial2.available()) {
      Serial2.read();
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    Serial2.write(addr);
    Serial2.write(func);
    Serial2.write(param);
    Serial2.write(crc);
    
    xSemaphoreGive(serial2Mutex);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
    return true;
  }
  return false;
}

// Get expected bytes for command
int getExpectedBytes(uint8_t command) {
  for (int i = 0; i < sizeof(expectedBytesMap) / sizeof(expectedBytesMap[0]); i++) {
    if (expectedBytesMap[i][0] == command) {
      return expectedBytesMap[i][1];
    }
  }
  return 3; // Default for status commands (addr + status + crc)
}

// Thread-safe feedback receiving
bool receiveFeedback(uint8_t addr, uint8_t command, FeedbackData* feedback) {
  if (xSemaphoreTake(serial2Mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }
  
  bool success = false;
  TickType_t startTime = xTaskGetTickCount();
  int expectedBytes = getExpectedBytes(command);

  // Wait for data with timeout
  while (Serial2.available() < expectedBytes && 
         (xTaskGetTickCount() - startTime) < pdMS_TO_TICKS(RESPONSE_TIMEOUT)) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (Serial2.available() >= expectedBytes) {
    uint8_t receivedAddr = Serial2.read();
    if (receivedAddr == addr) {
      feedback->success = true;
      
      if (command == READ_PULSE) {
        uint8_t data[4];
        for (int i = 0; i < 4; i++) {
          data[i] = Serial2.read();
        }
        uint8_t crc = Serial2.read();
        uint8_t calculatedCRC = calculateDataCRC(receivedAddr, data, 4);
        
        if (crc == calculatedCRC) {
          uint32_t pulse = 0;
          for (int i = 0; i < 4; i++) {
            pulse |= (uint32_t)data[i] << (24 - i * 8);
          }
          feedback->pulseData.pulse = pulse;
          feedback->pulseData.degrees = (float)pulse * FULL_CIRCLE / MOTOR_STEPS;
          success = true;
        }
      } 
      else if (command == READ_ENCODER) {
        uint8_t carryData[4], valueData[2];
        for (int i = 0; i < 4; i++) carryData[i] = Serial2.read();
        for (int i = 0; i < 2; i++) valueData[i] = Serial2.read();
        uint8_t crc = Serial2.read();
        
        uint8_t allData[6];
        memcpy(allData, carryData, 4);
        memcpy(allData + 4, valueData, 2);
        uint8_t calculatedCRC = calculateDataCRC(receivedAddr, allData, 6);
        
        if (crc == calculatedCRC) {
          int32_t carry = 0;
          for (int i = 0; i < 4; i++) {
            carry |= (int32_t)carryData[i] << (24 - i * 8);
          }
          uint16_t value = 0;
          for (int i = 0; i < 2; i++) {
            value |= (uint16_t)valueData[i] << (8 - i * 8);
          }
          
          feedback->encoderData.carry = carry;
          feedback->encoderData.value = value;
          float currentAngleInRevolution = (float)value * FULL_CIRCLE / ENCODER_RESOLUTION;
          feedback->encoderData.totalAngle = (float)carry * FULL_CIRCLE + currentAngleInRevolution;
          feedback->encoderData.displayAngle = fmod(feedback->encoderData.totalAngle, FULL_CIRCLE);
          if (feedback->encoderData.displayAngle < 0) 
            feedback->encoderData.displayAngle += FULL_CIRCLE;
          success = true;
        }
      }
      else if (command == READ_ERROR) {
        uint8_t data[2];
        for (int i = 0; i < 2; i++) data[i] = Serial2.read();
        uint8_t crc = Serial2.read();
        uint8_t calculatedCRC = calculateDataCRC(receivedAddr, data, 2);
        
        if (crc == calculatedCRC) {
          uint16_t value = 0;
          for (int i = 0; i < 2; i++) {
            value |= (uint16_t)data[i] << (8 - i * 8);
          }
          feedback->errorData.error = (int16_t)value;
          feedback->errorData.degrees = (float)feedback->errorData.error * FULL_CIRCLE / ENCODER_RESOLUTION;
          success = true;
        }
      }
      // Handle status responses for configuration commands
      else {
        uint8_t status = Serial2.read();
        uint8_t crc = Serial2.read();
        uint8_t calculatedCRC = calculateDataCRC(receivedAddr, &status, 1);
        
        if (crc == calculatedCRC) {
          feedback->statusData.status = status;
          success = true;
        }
      }
    }
    
    // Clear remaining data
    while (Serial2.available()) Serial2.read();
  }
  
  feedback->success = success;
  xSemaphoreGive(serial2Mutex);
  return success;
}

// Thread-safe serial print
void safePrint(const char* message) {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.print(message);
    xSemaphoreGive(serialMutex);
  }
}

void safePrintln(const char* message) {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println(message);
    xSemaphoreGive(serialMutex);
  }
}

// FreeRTOS Tasks
void motorControlTask(void* parameter) {
  TickType_t lastPrintTime = 0;
  bool allDone = false;
  
  while (true) {
    unsigned wait_time = controller.nextAction();
    
    if (wait_time) {
      // Motors are running
      if ((xTaskGetTickCount() - lastPrintTime) >= pdMS_TO_TICKS(PRINT_INTERVAL)) {
        for (int i = 0; i < 3; i++) {
          if (steppers[i].getStepsRemaining()) {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
              Serial.print("Motor ");
              Serial.print(i == 0 ? "X" : i == 1 ? "Y" : "Z");
              Serial.print(": ");
              Serial.print(steppers[i].getStepsCompleted());
              Serial.print("/");
              Serial.print(steppers[i].getStepsCompleted() + steppers[i].getStepsRemaining());
              Serial.print(" steps, ");
              Serial.print(steppers[i].getCurrentRPM(), 1);
              Serial.println(" RPM");
              xSemaphoreGive(serialMutex);
            }
          }
        }
        lastPrintTime = xTaskGetTickCount();
      }
      allDone = false;
    } 
    else if (!allDone) {
      // Motors finished
      for (int i = 0; i < 3; i++) {
        steppers[i].disable();
      }
      safePrintln("\n=== All motors stopped ===");
      allDone = true;
      
      // Start restart timer
      xTimerStart(restartTimer, 0);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void uartFeedbackTask(void* parameter) {
  FeedbackData feedback;
  TickType_t lastFeedbackTime = 0;
  
  while (true) {
    if ((xTaskGetTickCount() - lastFeedbackTime) >= pdMS_TO_TICKS(FEEDBACK_INTERVAL)) {
      safePrintln("\n=== READING FEEDBACK ===");
      
      for (int i = 0; i < 3; i++) {
        feedback.motorIndex = i;
        
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          Serial.print("--- Module ");
          Serial.print(i == 0 ? "X" : i == 1 ? "Y" : "Z");
          Serial.println(" ---");
          xSemaphoreGive(serialMutex);
        }
        
        // Read pulse
        feedback.command = READ_PULSE;
        if (sendCommand(motorConfigs[i].address, READ_PULSE)) {
          if (receiveFeedback(motorConfigs[i].address, READ_PULSE, &feedback)) {
            xQueueSend(feedbackQueue, &feedback, 0);
          }
        }
        
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
        
        // Read encoder
        feedback.command = READ_ENCODER;
        if (sendCommand(motorConfigs[i].address, READ_ENCODER)) {
          if (receiveFeedback(motorConfigs[i].address, READ_ENCODER, &feedback)) {
            xQueueSend(feedbackQueue, &feedback, 0);
          }
        }
        
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
        
        // Read error
        feedback.command = READ_ERROR;
        if (sendCommand(motorConfigs[i].address, READ_ERROR)) {
          if (receiveFeedback(motorConfigs[i].address, READ_ERROR, &feedback)) {
            xQueueSend(feedbackQueue, &feedback, 0);
          }
        }
        
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
      }
      
      lastFeedbackTime = xTaskGetTickCount();
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void serialPrintTask(void* parameter) {
  FeedbackData feedback;
  
  while (true) {
    if (xQueueReceive(feedbackQueue, &feedback, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        char motorName = feedback.motorIndex == 0 ? 'X' : 
                        feedback.motorIndex == 1 ? 'Y' : 'Z';
        
        if (feedback.command == READ_PULSE && feedback.success) {
          Serial.print("Stepper ");
          Serial.print(motorName);
          Serial.print(" Pulse: ");
          Serial.print(feedback.pulseData.pulse);
          Serial.print(" = ");
          Serial.print(feedback.pulseData.degrees, 2);
          Serial.println("°");
        }
        else if (feedback.command == READ_ENCODER && feedback.success) {
          Serial.print("Stepper ");
          Serial.print(motorName);
          Serial.print(" Encoder: ");
          Serial.print(feedback.encoderData.displayAngle, 1);
          Serial.print("° (");
          Serial.print(feedback.encoderData.carry);
          Serial.println(" revs)");
        }
        else if (feedback.command == READ_ERROR && feedback.success) {
          Serial.print("Stepper ");
          Serial.print(motorName);
          Serial.print(" Error: ");
          Serial.print(feedback.errorData.degrees, 2);
          Serial.println("°");
        }
        else if (feedback.success) {
          // For configuration commands
          Serial.print("Command 0x");
          Serial.print(feedback.command, HEX);
          Serial.print(" Status: ");
          Serial.println(feedback.statusData.status == 1 ? "Success" : "Failed");
        }
        
        xSemaphoreGive(serialMutex);
      }
    }
  }
}

// Timer callback for restart
void restartTimerCallback(TimerHandle_t xTimer) {
  safePrintln("\n=== Restarting rotation ===");
  
  for (int i = 0; i < 3; i++) {
    steppers[i].enable();
  }
  controller.rotate(ROTATION_ANGLE, ROTATION_ANGLE, ROTATION_ANGLE);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  // Initialize steppers
  for (int i = 0; i < 3; i++) {
    steppers[i].begin(motorConfigs[i].rpm, MICROSTEPS);
    steppers[i].setEnableActiveState(LOW);
    steppers[i].enable();
    steppers[i].setSpeedProfile(steppers[i].LINEAR_SPEED, 1000, 2000);
  }

  // Create FreeRTOS objects
  serial2Mutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  feedbackQueue = xQueueCreate(10, sizeof(FeedbackData));
  restartTimer = xTimerCreate("RestartTimer", pdMS_TO_TICKS(RESTART_DELAY), 
                              pdFALSE, (void*)0, restartTimerCallback);

  if (serial2Mutex == NULL || serialMutex == NULL || 
      feedbackQueue == NULL || restartTimer == NULL) {
    Serial.println("Failed to create FreeRTOS objects!");
    while(1);
  }

  Serial.println("START - 3 motors ready");
  Serial.println("Resetting encoders...");
  
  // Reset encoders - Fixed command code based on config
  for (int i = 0; i < 3; i++) {
    // Use SET_ZERO_POINT instead of wrong RESET_ENCODER
    sendCommandWithParam(motorConfigs[i].address, 0x91, 0x00);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
  }

  Serial.println("Setting CR_vFOC mode...");
  for (int i = 0; i < 3; i++) {
    // Set work mode to CR_vFOC (mode 2)
    sendCommandWithParam(motorConfigs[i].address, SET_CR_VFOC, 0x02);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY));
  }

  Serial.println("Starting rotation...");
  controller.rotate(ROTATION_ANGLE, ROTATION_ANGLE, ROTATION_ANGLE);

  // Create tasks
  xTaskCreatePinnedToCore(motorControlTask, "MotorControl", 4096, NULL, 2, 
                         &motorControlTaskHandle, 1);
  xTaskCreatePinnedToCore(uartFeedbackTask, "UARTFeedback", 4096, NULL, 2, 
                         &uartFeedbackTaskHandle, 0);
  xTaskCreatePinnedToCore(serialPrintTask, "SerialPrint", 2048, NULL, 1, 
                         &serialPrintTaskHandle, 0);

  Serial.println("FreeRTOS tasks started!");
}

void loop() {
  // Empty - all work done by FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}