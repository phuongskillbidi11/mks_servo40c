#include "config1.h"
#include <Arduino.h>


#ifndef TEST_AccelTest 

#include <Arduino.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 3200
// Target RPM for cruise speed
#define RPM 40
// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 1000
#define MOTOR_DECEL 2000

// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 1

#define DIR 25
#define STEP 26
#define SLEEP 27 // optional (just delete SLEEP from everywhere if not used)

/*
 * Choose one of the sections below that match your board
 */

// #include "DRV8834.h"
// #define M0 10
// #define M1 11
// DRV8834 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, M0, M1);

// #include "A4988.h"
// #define MS1 10
// #define MS2 11
// #define MS3 12
// A4988 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, MS1, MS2, MS3);

#include "DRV8825.h"
#define MODE0 -1
#define MODE1 -1
#define MODE2 -1
DRV8825 stepper(MOTOR_STEPS, DIR, STEP);

void setup() {
    Serial.begin(115200);

    stepper.begin(RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    stepper.setEnableActiveState(LOW);
    stepper.enable();
    // set current level (for DRV8880 only). Valid percent values are 25, 50, 75 or 100.
    // stepper.setCurrent(100);

    /*
     * Set LINEAR_SPEED (accelerated) profile.
     */
    stepper.setSpeedProfile(stepper.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);

    Serial.println("START");
    /*
     * Using non-blocking mode to print out the step intervals.
     * We could have just as easily replace everything below this line with 
     * stepper.rotate(360);
     */
     stepper.startRotate(180);
}

void loop() {
    static int step = 0;
    unsigned wait_time = stepper.nextAction();
    if (wait_time){
        Serial.print("  step="); Serial.print(step++);
        Serial.print("  dt="); Serial.print(wait_time);
        Serial.print("  rpm="); Serial.print(stepper.getCurrentRPM());
        Serial.println();

    } else {
        stepper.disable();
        Serial.println("END");
        delay(3600);
    }
}
#endif ///TEST_AccelTest 

#ifndef TEST_BasicStepperDriver
#include <Arduino_JSON.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"

#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 2100
// #define RPM 70
#define RPM 20

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// All the wires needed for full functionality
#define DIR 25
#define STEP 26
//Uncomment line to use enable/disable functionality
//#define SLEEP 13

// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

//Uncomment line to use enable/disable functionality
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, SLEEP);

void setup() {
    stepper.begin(RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);
}

void loop() {
  
    // energize coils - the motor will hold position
    // stepper.enable();
  
    /*
     * Moving motor one full revolution using the degree notation
     */
    stepper.rotate(360);

    /*
     * Moving motor to original position using steps
     */
    stepper.move(-MOTOR_STEPS*MICROSTEPS);

    // pause and allow the motor to be moved by hand
    // stepper.disable();

    delay(1000);
}

#endif ///TESTBasicStepperDriver

#ifndef control_motorstep_rpm
#include <Arduino.h>
#include <Arduino_JSON.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"


#include "BasicStepperDriver.h"
#define BOOT_PIN 4
////set up value ///
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
// #define MOTOR_STEPS 2100
// // #define RPM 70
// #define RPM 20
int MOTOR_STEPS = 2100;
int RPM = 20;


/////ssid_pass////
const char* ssid = "I-Soft";
const char* password = "i-soft@2023";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


unsigned long lastTime = 0;
unsigned long timerDelay = 500;

volatile bool motorEnabled = true;

// Initialize LittleFS
void initLittleFS() {
  if (!LittleFS.begin(true)) {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  Serial.println("LittleFS mounted successfully");
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}


void setMotorParameters(int rpm, int Motor_step) {
  RPM = rpm;//20 -> 120
  MOTOR_STEPS = Motor_step;/// 1000...21000
}


    ///Pin //
// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// All the wires needed for full functionality
#define DIR 25
#define STEP 26
//Uncomment line to use enable/disable functionality
//#define SLEEP 13


BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

long currentTime = millis();
bool SocketConnected = false;

String getMotorStepperReadings(int set_rpm, int from_step, int to_step,int type ){
  //   String jsonString;
    JSONVar myObject;
  //   if (type = 0){/// TOPIC BASICSTEPPER
  //       myObject["RPM"] [set_rpm]= set_rpm;
        
  //   for (int Motor_step = from_step; Motor_step < to_step; Motor_step += 1000  ){
  //     myObject ["MOTOR_STEPS"][Motor_step] = Motor_step;
  //   }
  // }  
  // jsonString = JSON.stringify(myObject);
  // Serial.println(jsonString);
   
    long stepsCompleted = stepper.getStepsCompleted();
    int direction = stepper.getDirection();
    float currentRPM = stepper.getCurrentRPM();
    

    float currentAngle = (stepsCompleted * 360.0) / (MOTOR_STEPS * MICROSTEPS);
    

    myObject["steps_completed"] = stepsCompleted;
    myObject["current_angle"] = currentAngle;
    myObject["direction"] = direction;
    myObject["current_rpm"] = currentRPM;
    myObject["target_rpm"] = set_rpm;
    myObject["from_step"] = from_step;
    myObject["to_step"] = to_step;
    myObject["motor_steps"] = MOTOR_STEPS;
    myObject["microsteps"] = MICROSTEPS;

    String jsonString = JSON.stringify(myObject);
    Serial.println("Motor Status: " + jsonString);
    return jsonString;
}

void notifyClients(String StepperReadings) {
  ws.textAll(StepperReadings);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len){
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String message = (char*)data;
    Serial.println(message);
    JSONVar JsonIn = JSON.parse(message.c_str());
  
   if (String((const char*)JsonIn["CMD"]) == "get"&& motorEnabled) {
      Serial.println("Set_PRM : From >> To  : "  " PRM: "+ String((int)JsonIn["Set_PRM"]) + " From: " + String((int)JsonIn["from"]) + " to_step: " +  String((int)JsonIn["to"])+ " type: " +  String((int)JsonIn["type"]));
      // String StepperReadings = getMotorStepperReadings((int)JsonIn["Set_PRM"],(int)JsonIn["from"], (int)JsonIn["to"], (int)JsonIn["type"]);
      // Serial.println(StepperReadings);
      // Serial.println("SocketConnected: " + String(SocketConnected));
      // if(SocketConnected){notifyClients(StepperReadings);}
      
        stepper.begin((int)JsonIn["Set_PRM"], MICROSTEPS);
        stepper.move((int)JsonIn["from"]);
        stepper.move((int)JsonIn["to"]);
        // stepper.begin((int)JsonIn["Set_PRM"], MICROSTEPS);
        // stepper.move((int)JsonIn["from"]);
        // stepper.move((int)JsonIn["to"]);
           String StepperReadings = getMotorStepperReadings(
                (int)JsonIn["Set_PRM"],
                (int)JsonIn["from"], 
                (int)JsonIn["to"], 
                (int)JsonIn["type"]
            );
            
        }

      }
    }

 void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      SocketConnected = 1;
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      SocketConnected = 0;
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}
 
void setup (){
  Serial.begin(115200);

  pinMode(BOOT_PIN, INPUT_PULLUP);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);

   ////websocket///
  initWiFi();Serial.println("init Wifi");
  initLittleFS();Serial.println("init FS");
  initWebSocket();Serial.println("init Socket");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(LittleFS, "/index.html", "text/html");
   });
    server.serveStatic("/", LittleFS, "/");
    server.begin();

  ///Stepper_Driver///
  stepper.begin(RPM, MICROSTEPS);


}
void loop(){
  if (digitalRead(BOOT_PIN) == LOW) {
    motorEnabled = false;
    digitalWrite(STEP, LOW);
    digitalWrite(DIR, LOW);
    stepper.stop();
     while (digitalRead(BOOT_PIN) == LOW) {
    }
    motorEnabled = true;
  }



  if((millis() - lastTime) > timerDelay){
    lastTime = millis();
  }
  ws.cleanupClients();
  // delay(1000); 
  // stepper.rotate(360);
  // stepper.rotate(-360);
  // stepper.move(MOTOR_STEPS*MICROSTEPS);
  // stepper.move(-MOTOR_STEPS*MICROSTEPS);
  //  delay(1000);

}
#endif //control_motorstep_rpm

#ifndef area

#include "BasicStepperDriver.h"
#define BOOT_PIN 4
////set up value ///
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
// #define MOTOR_STEPS 2100
// // #define RPM 70
// #define RPM 20
int MOTOR_STEPS = 2100;
int RPM = 20;


/////ssid_pass////
const char* ssid = "I-Soft";
const char* password = "i-soft@2023";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


unsigned long lastTime = 0;
unsigned long timerDelay = 500;

volatile bool motorEnabled = true;

// Initialize LittleFS
void initLittleFS() {
  if (!LittleFS.begin(true)) {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  Serial.println("LittleFS mounted successfully");
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}


void setMotorParameters(int rpm, int Motor_step) {
  RPM = rpm;//20 -> 120
  MOTOR_STEPS = Motor_step;/// 1000...21000
}


    ///Pin //
// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// All the wires needed for full functionality
#define DIR 25
#define STEP 26
//Uncomment line to use enable/disable functionality
//#define SLEEP 13


BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

long currentTime = millis();
bool SocketConnected = false;

String getMotorStepperReadings(int set_rpm, int from_step, int to_step,int type ){
  //   String jsonString;
    JSONVar myObject;
  //   if (type = 0){/// TOPIC BASICSTEPPER
  //       myObject["RPM"] [set_rpm]= set_rpm;
        
  //   for (int Motor_step = from_step; Motor_step < to_step; Motor_step += 1000  ){
  //     myObject ["MOTOR_STEPS"][Motor_step] = Motor_step;
  //   }
  // }  
  // jsonString = JSON.stringify(myObject);
  // Serial.println(jsonString);
   
    // long stepsCompleted = stepper.getStepsCompleted();
    // int direction = stepper.getDirection();
    // float currentRPM = stepper.getCurrentRPM();
    

    // float currentAngle = (stepsCompleted * 360.0) / (MOTOR_STEPS * MICROSTEPS);
    

    // myObject["steps_completed"] = stepsCompleted;
    // myObject["current_angle"] = currentAngle;
    // myObject["direction"] = direction;
    // myObject["current_rpm"] = currentRPM;
    // myObject["target_rpm"] = set_rpm;
    // myObject["from_step"] = from_step;
    // myObject["to_step"] = to_step;
    // myObject["motor_steps"] = MOTOR_STEPS;
    // myObject["microsteps"] = MICROSTEPS;

    // String jsonString = JSON.stringify(myObject);
    // Serial.println("Motor Status: " + jsonString);
    // return jsonString;
}

void notifyClients(String StepperReadings) {
  ws.textAll(StepperReadings);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len){
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String message = (char*)data;
    Serial.println(message);
    JSONVar JsonIn = JSON.parse(message.c_str());
  
   if (String((const char*)JsonIn["CMD"]) == "get"&& motorEnabled) {
             /////////////////
    #ifndef Sensor Data Dashboard
      Serial.println("Set_PRM : From >> To  : "  " PRM: "+ String((int)JsonIn["Set_PRM"]) + " From: " + String((int)JsonIn["from"]) + " to_step: " +  String((int)JsonIn["to"])+ " type: " +  String((int)JsonIn["type"]));
      // String StepperReadings = getMotorStepperReadings((int)JsonIn["Set_PRM"],(int)JsonIn["from"], (int)JsonIn["to"], (int)JsonIn["type"]);
      // Serial.println(StepperReadings);
      // Serial.println("SocketConnected: " + String(SocketConnected));
      // if(SocketConnected){notifyClients(StepperReadings);}
      
        stepper.begin((int)JsonIn["Set_PRM"], MICROSTEPS);
        stepper.move((int)JsonIn["from"]);
        stepper.move((int)JsonIn["to"]);
        // stepper.begin((int)JsonIn["Set_PRM"], MICROSTEPS);
        // stepper.move((int)JsonIn["from"]);
        // stepper.move((int)JsonIn["to"]);
          //  String StepperReadings = getMotorStepperReadings(
          //       (int)JsonIn["Set_PRM"],
          //       (int)JsonIn["from"], 
          //       (int)JsonIn["to"], 
          //       (int)JsonIn["type"]
          //   );
         #endif //Sensor Data Dashboard
             /////////////////

    String device = (const char*)JsonIn["Device"];
    int step = (int)JsonIn["Step"];
    float degree = String((const char*)JsonIn["Degree"]).toFloat();
    int set_prm = (int)JsonIn["Set_PRM"];
    #ifndef Control MultiDriver Stepper Dashboard
    Serial.println("CMD: get | Device: " + device + " | Step: " + String(step) + " | Degree: " + String(degree, 2) + " | Set_PRM: " + String(set_prm));
        stepper.begin(set_prm, MICROSTEPS);
        stepper.move(degree);
         #endif //Control MultiDriver Stepper Dashboard
        }  
      }
    }
 
    

 void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      SocketConnected = 1;
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      SocketConnected = 0;
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}
 
void setup (){
  Serial.begin(115200);

  pinMode(BOOT_PIN, INPUT_PULLUP);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);

   ////websocket///
  initWiFi();Serial.println("init Wifi");
  initLittleFS();Serial.println("init FS");
  initWebSocket();Serial.println("init Socket");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(LittleFS, "/index.html", "text/html");
   });
    server.serveStatic("/", LittleFS, "/");
    server.begin();

  ///Stepper_Driver///
  stepper.begin(RPM, MICROSTEPS);


}
void loop(){
  if (digitalRead(BOOT_PIN) == LOW) {
    motorEnabled = false;
    digitalWrite(STEP, LOW);
    digitalWrite(DIR, LOW);
    stepper.stop();
     while (digitalRead(BOOT_PIN) == LOW) {
    }
    motorEnabled = true;
  }



  if((millis() - lastTime) > timerDelay){
    lastTime = millis();
  }
  ws.cleanupClients();
  // delay(1000); 
  // stepper.rotate(360);
  // stepper.rotate(-360);
  // stepper.move(MOTOR_STEPS*MICROSTEPS);
  // stepper.move(-MOTOR_STEPS*MICROSTEPS);
  //  delay(1000);

}
#endif //area

