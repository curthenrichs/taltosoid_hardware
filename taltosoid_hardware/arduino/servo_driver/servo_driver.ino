#include <Servo.h>

#define BASE_SERVO_PIN (6)
#define MID_SERVO_PIN (9)
#define TIP_SERVO_PIN (10)


// Servo bounds (degrees)
#define BASE_DEG_MIN (20)
#define BASE_DEG_MAX (170)
#define MID_DEG_MIN (55)
#define MID_DEG_MAX (170)
#define TIP_DEG_MIN (20)
#define TIP_DEG_MAX (170)

// Servo default state
#define BASE_DEG_DEFAULT (160)
#define MID_DEG_DEFAULT (140)
#define TIP_DEG_DEFAULT (90)


Servo baseServo;
Servo midServo;
Servo tipServo;

int baseServoValue;
int midServoValue;
int tipServoValue;

char buffer[64];

void setup() {
 Serial.begin(115200);

 baseServo.attach(BASE_SERVO_PIN);
 midServo.attach(MID_SERVO_PIN);
 tipServo.attach(TIP_SERVO_PIN);

 tipServo.write(TIP_DEG_DEFAULT);
 tipServoValue = TIP_DEG_DEFAULT;
 midServo.write(MID_DEG_DEFAULT);
 midServoValue = MID_DEG_DEFAULT;
 baseServo.write(BASE_DEG_DEFAULT);
 baseServoValue = BASE_DEG_DEFAULT;

 Serial.println("Setup Complete");
}

void loop() {

  // Capture serial data
  memset(buffer,0,64);
  int len = Serial.readBytesUntil('\n', buffer, 64);

  // Run CLI for servo driver
  if (len != 0) {
    int mode, servoId, value;
    if(sscanf(buffer,"%d:%d:%d",&mode,&servoId,&value) != 3) {
      // invalid format
      Serial.println("Invalid command form, use <'MODE'>:<'ID'>:<'VALUE'>");
    } else if (mode == 0) {
      if (servoId == 0) {
        if (boundCheck(value,BASE_DEG_MIN,BASE_DEG_MAX)) {
          baseServo.write(value);
          baseServoValue = value;
          Serial.print("Set Servo 0 to ");
          Serial.println(value);
        } else {
          Serial.println("Invalid bound for Servo 0");
        }
      } else if (servoId == 1) {
        if (boundCheck(value,MID_DEG_MIN,MID_DEG_MAX)) {
          midServo.write(value);
          midServoValue = value;
          Serial.print("Set Servo 1 to ");
          Serial.println(value);
        } else {
          Serial.println("Invalid bound for Servo 1");
        }
      } else if (servoId == 2) {
        if(boundCheck(value,TIP_DEG_MIN,TIP_DEG_MAX)) {
          tipServo.write(value);
          tipServoValue = value;
          Serial.print("Set Servo 2 to ");
          Serial.println(value);
        } else {
          Serial.println("Invalid bound for Servo 2");
        }
      } else {
        // invalid servo id
        Serial.println("Invalid Servo ID");
      }
    } else if (mode == 1) {
      if (servoId == 0) {
        Serial.print("Get Servo 0 ");
        Serial.println(baseServoValue);
      } else if (servoId == 1) {
        Serial.print("Get Servo 1 ");
        Serial.println(midServoValue);
      } else if (servoId == 2) {
        Serial.print("Get Servo 2 ");
        Serial.println(tipServoValue);
      } else {
        // invalid servo id
        Serial.println("Invalid Servo ID");
      }
    } else {
      //invalid mode
      Serial.println("Invalid Mode");
    }
  }

  delay(50);

}

bool boundCheck(int value, int min, int max) {
  return value >= min && value <= max;
}
