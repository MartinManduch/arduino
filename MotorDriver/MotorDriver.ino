// One Arduino Leonardo has 2 motor drivers connected, each is driving 2 motors, so together 4 motors
// Note: if motors is close to supplied with 100% of 24V i.e. PWM is 255, than even if we stop motor, a few sensor impulses could be produced, it can be max 1,2 for super power jack, but for smaller one it could be up to 5, but for the sake of simplicity, we are not counting them
#include "consts.h"
#include "utils.h"

volatile unsigned long currentImpulses = 0;
unsigned long targetImpulses = 0;
byte speed = 0;
byte command = 0;
boolean enableInterrupts = false; // we need a special variable, because calling attachInterrupt causes immediate interrupt

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // enable D2 - required for drivers to work
  pinMode(DRIVER1_D2_PIN, OUTPUT);
  digitalWrite(DRIVER1_D2_PIN, HIGH);
  pinMode(DRIVER2_D2_PIN, OUTPUT);
  digitalWrite(DRIVER2_D2_PIN, HIGH);

  pinMode(MOTOR1_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR1_SPEED_PIN, OUTPUT);
  analogWrite(MOTOR1_SPEED_PIN, 0);
  digitalWrite(MOTOR1_SENSOR_PIN, HIGH);

  pinMode(MOTOR2_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR2_SPEED_PIN, OUTPUT);
  analogWrite(MOTOR2_SPEED_PIN, 0);
  digitalWrite(MOTOR2_SENSOR_PIN, HIGH);

  pinMode(MOTOR3_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR3_SPEED_PIN, OUTPUT);
  analogWrite(MOTOR3_SPEED_PIN, 0);
  pinMode(MOTOR3_SENSOR_PIN, INPUT_PULLUP);

  pinMode(MOTOR4_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR4_SPEED_PIN, OUTPUT);
  analogWrite(MOTOR4_SPEED_PIN, 0);
  pinMode(MOTOR4_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(mapInterrupt(MOTOR1_SENSOR_PIN), interrupt_handler1, RISING);
  attachInterrupt(mapInterrupt(MOTOR2_SENSOR_PIN), interrupt_handler2, RISING);
  enableInterrupts = true;
}

// the loop function runs over and over again forever
void loop() {

  readSerialForInput();

  switch (command) {
    case 1:
      disableMotors();
      break;
    case 2:
      enableMotors();
      break;
    case 10:
      goForwardSmallMotor1();
      reset();
      break;
    case 11:
      goBackSmallMotor1();
      reset();
      break;
    case 30:
      goForwardBigMotor1();
      reset();
      break;
    case 31:
      goBackBigMotor1();
      reset();
      break;
    case 40:
      goForwardBigMotor1();
      reset();
      break;
    case 41:
      goBackBigMotor1();
      reset();
      break;
  }
  delay(SERIAL_INPUT_DELAY); // TODO: make as constant
}


void goForwardSmallMotor1() {
  digitalWrite(MOTOR1_DIRECTION_PIN, LOW);
  analogWrite(MOTOR1_SPEED_PIN, speed);
  checkProgress(MOTOR1_SPEED_PIN);
}

void goBackSmallMotor1() {
  digitalWrite(MOTOR1_DIRECTION_PIN, HIGH);
  analogWrite(MOTOR1_SPEED_PIN, speed);
  checkProgress(MOTOR1_SPEED_PIN);
}

void goForwardSmallMotor2() {
  digitalWrite(MOTOR2_DIRECTION_PIN, LOW);
  analogWrite(MOTOR2_SPEED_PIN, speed);
  checkProgress(MOTOR2_SPEED_PIN);
}

void goBackSmallMotor2() {
  digitalWrite(MOTOR2_DIRECTION_PIN, HIGH);
  analogWrite(MOTOR2_SPEED_PIN, speed);
  checkProgress(MOTOR2_SPEED_PIN);
}

void checkProgress(byte speedPIN) {
  int before;
  int after;
  while (currentImpulses < targetImpulses) {
    before = currentImpulses;
    delay(SENSOR1_TIMEOUT);
    if (before == currentImpulses && currentImpulses < targetImpulses) {
      Serial.println("eMotor stopped not reaching target");
      stopMotor(speedPIN);
      break;
    }
  }
}

void interrupt_handler1() {
  if (enableInterrupts) {
    currentImpulses++;
    Serial.print("i");
    Serial.println(currentImpulses, DEC);
    if (currentImpulses >= targetImpulses) {
      stopMotor(MOTOR1_SPEED_PIN);
    }
  }
}

void interrupt_handler2() {
  if (enableInterrupts) {
    currentImpulses++;
    Serial.print("i");
    Serial.println(currentImpulses, DEC);
    if (currentImpulses >= targetImpulses) {
      stopMotor(MOTOR2_SPEED_PIN);
    }
  }
}


void readSerialForInput() {
  command = getCommand();
  int param1 = 0;
  if (command > 0) {
    while (param1 == 0) {
      param1 = getParam1();
      delay(1);
    }
    int param2 = 0;
    while (param2 == 0) {
      param2 = getParam2();
      delay(1);
    }
    speed = param1;
    targetImpulses = param2;
    Serial.print("mCommand: ");
    Serial.print(command);
    Serial.print(" Param1: ");
    Serial.print(param1);
    Serial.print(" Param2: ");
    Serial.println(param2);

  }
}

byte getCommand() {
  if (Serial.available() > 0) {
    return Serial.read();
  }
  else {
    return 0;
  }
}

byte getParam1() {
  if (Serial.available() > 0) {
    return Serial.read();
  }
  else {
    return 0;
  }
}

unsigned long getParam2() {
  if (Serial.available() > 3) {
    return readULongFromBytes();
  }
  else {
    return 0;
  }
}

void disableMotors() {
  digitalWrite(DRIVER1_D2_PIN, LOW);
  digitalWrite(DRIVER2_D2_PIN, LOW);
}

void enableMotors() {
  reset();
}


void doWork(byte sensorPIN) {
  int value;
  boolean logicalValue = false;
  boolean previousLogicalValue = false;
  unsigned long previousTime = millis();
  while (currentImpulses < targetImpulses) {
    value = analogRead(sensorPIN);
    previousLogicalValue = logicalValue;
    if (value > CUSTOM_HIGH) {
      logicalValue = true;
    }
    if (value < CUSTOM_LOW) {
      logicalValue = false;
    }

    if ((logicalValue == false) && (previousLogicalValue == true)) {
      previousTime = millis();
      currentImpulses++;
      Serial.print("i");
      Serial.println(currentImpulses, DEC);

    }
    else if ((millis() - previousTime) > SENSOR2_TIMEOUT) { // no signal change last SENSOR2_TIMEOUT msec
      Serial.println("eMotor stopped not reaching target");
      break;
    }
    delay(SENSOR2_READ_FREQ);
  }
}

void goForwardBigMotor1() {
  digitalWrite(MOTOR3_DIRECTION_PIN, HIGH);
  analogWrite(MOTOR3_SPEED_PIN, speed);
  doWork(MOTOR3_SENSOR_PIN);
  stopMotor(MOTOR3_SPEED_PIN);
}

void goBackBigMotor1() {
  digitalWrite(MOTOR3_DIRECTION_PIN, LOW);
  analogWrite(MOTOR3_SPEED_PIN, speed);
  doWork(MOTOR3_SENSOR_PIN);
  stopMotor(MOTOR3_SPEED_PIN);
}

void goForwardBigMotor2() {
  digitalWrite(MOTOR4_DIRECTION_PIN, HIGH);
  analogWrite(MOTOR4_SPEED_PIN, speed);
  doWork(MOTOR4_SENSOR_PIN);
  stopMotor(MOTOR4_SPEED_PIN);
}

void goBackBigMotor2() {
  digitalWrite(MOTOR4_DIRECTION_PIN, LOW);
  analogWrite(MOTOR4_SPEED_PIN, speed);
  doWork(MOTOR4_SENSOR_PIN);
  stopMotor(MOTOR4_SPEED_PIN);
}

inline void stopMotor(byte motorSpeedPin) {
  analogWrite(motorSpeedPin, 0);
  Serial.println("mSTOP");
}

inline void reset() { // sets variables to the default state, also stops all motors by setting speed to 0
  currentImpulses = 0;
  targetImpulses = 0;
  command = 0;
  speed = 0;

  digitalWrite(DRIVER1_D2_PIN, HIGH);
  digitalWrite(DRIVER2_D2_PIN, HIGH);
  analogWrite(MOTOR1_SPEED_PIN, 0);
  digitalWrite(MOTOR1_SENSOR_PIN, HIGH);
  analogWrite(MOTOR2_SPEED_PIN, 0);
  digitalWrite(MOTOR2_SENSOR_PIN, HIGH);
  analogWrite(MOTOR3_SPEED_PIN, 0);
  analogWrite(MOTOR4_SPEED_PIN, 0);
}

