#include <SoftwareSerial.h>  // library for bluetooth

#include <Wire.h>                                // library for motorshield
#include <Adafruit_MotorShield.h>                // library for motorshield
#include "utility/Adafruit_MS_PWMServoDriver.h"  // library for motorshield

// create new arduino shield object
Adafruit_MotorShield arduinoShield = Adafruit_MotorShield();

// motor setup
Adafruit_DCMotor *leftThruster = arduinoShield.getMotor(4);
Adafruit_DCMotor *rightThruster = arduinoShield.getMotor(1);
Adafruit_DCMotor *conveyorBelt = arduinoShield.getMotor(3);
Adafruit_DCMotor *conveyorBeltTilt = arduinoShield.getMotor(2);
int last = 0;

// bluetooth setup
SoftwareSerial MyBlue(2, 5);  // set RX and TX
String msg = "";              //used for bluetooth commands

// ping sensor setup
int distance2pin = 11;
int distance3pin = 10;

void setup() {

  Serial.begin(9600);  // set up serial monitor
  MyBlue.begin(9600);  // set up bluetooth serial monitor

  arduinoShield.begin();  // set up motor shield

}

void loop() {

  checkDistance();
  motorControl();

}

// method that controls motors
void motorControl() {

  // if connected with bluetooth
  if (MyBlue.available() > 0);

  // get input from bluetooth
  msg = MyBlue.readString();

  // truncate to only have useful information
  msg = msg.substring(0, msg.length() - 1);
  msg = msg.substring(msg.lastIndexOf(" ") + 1);

  // find which motor is being moved (LT for left thruster, RT fro right thruster CT for conveyor belt tilt, CB for conveyor belt)
  String type = msg.substring(0, 2);

  // map the 0 to 100 seekBar value to the 0 to 255 motor values
  int val = 255 * (msg.substring(2)).toInt() / 100;

  // move appropriate motors given command
  if (type.equals("LT")) {
    leftThruster->setSpeed(val);
    leftThruster->run(FORWARD);

  } else if (type.equals("RT")) {
    rightThruster->setSpeed(val);
    rightThruster->run(FORWARD);

  } else if (type.equals("CT")) {

    int x = map(val, 0, 255, 0, 3);

    // change position of conveyor belt depending on current position
    if (x > last) {
      for (int i = last; i < x; i++) {
        conveyorBeltTilt->setSpeed(500);
        conveyorBeltTilt->run(BACKWARD);
        delay(1000);
      }
      conveyorBeltTilt->setSpeed(0);
      conveyorBeltTilt->run(FORWARD);

    } else if (x < last) {
      for (int i = last; i >= x; i--) {
        conveyorBeltTilt->setSpeed(500);
        conveyorBeltTilt->run(FORWARD);
        delay(1000);
      }
      conveyorBeltTilt->setSpeed(0);
      conveyorBeltTilt->run(FORWARD);
    }
    last = x;

  } else if (type.equals("CB")) {
    conveyorBelt->setSpeed(val);
    conveyorBelt->run(FORWARD);
  }
}

// method to check ultrasonic sensors
void checkDistance() {

  int distanceThreshold = 20;

  // measure the ping time in centimeters and convert to inches
  int inchesRight = (0.01723 * readUltrasonicDistance(distance2pin) / 2.54);
  int inchesLeft = (0.01723 * readUltrasonicDistance(distance3pin) / 2.54);

  warning(inchesLeft, "l");
  delay(10);
  warning(inchesRight, "r");
}

// method to get input from ultrasonic sensor
long readUltrasonicDistance(int sig) {

  pinMode(sig, OUTPUT);  // Clear the trigger
  digitalWrite(sig, LOW);
  delayMicroseconds(2);

  // Sets the trigger pin to HIGH state for 10 microseconds
  digitalWrite(sig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sig, LOW);
  pinMode(sig, INPUT);

  // Reads the echo pin, and returns the sound wave travel time in microseconds
  return pulseIn(sig, HIGH);
}

// output warning to app
void warning(int inches, String d) {

  int distanceThreshold = 20;

  if (inches > distanceThreshold) {
    MyBlue.println(d + "is safe!");
  }
  if (inches <= distanceThreshold && inches > distanceThreshold - 15) {
    MyBlue.println(d + "is close to an object!");
  }
  if (inches < distanceThreshold - 15) {
    MyBlue.println(d + "is in danger!");
  }
}
