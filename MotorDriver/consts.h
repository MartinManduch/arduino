#include "Arduino.h"

const byte MOTOR1_SENSOR_PIN = 0; // must be interrupt, 
const byte MOTOR1_SPEED_PIN = 3; //must be PWM
const byte MOTOR1_DIRECTION_PIN = 2;

const byte MOTOR2_SENSOR_PIN = 1; // must be interrupt
const byte MOTOR2_SPEED_PIN = 5; //must be PWM
const byte MOTOR2_DIRECTION_PIN = 4; 

const byte MOTOR3_SENSOR_PIN = A0;  //sensor is just periodic analogRead
const byte MOTOR3_SPEED_PIN = 6; //must be PWM
const byte MOTOR3_DIRECTION_PIN = 7; 

const byte MOTOR4_SENSOR_PIN = A1; //sensor is just periodic analogRead
const byte MOTOR4_SPEED_PIN = 9; //must be PWM
const byte MOTOR4_DIRECTION_PIN = 8; 

const byte DRIVER1_D2_PIN = 10; // which pin of arduino is connected to driver D2 PIN (quick turn off)
const byte DRIVER2_D2_PIN = 11;  // which pin of arduino is connected to driver D2 PIN (quick turn off) , for the second driver


const int CUSTOM_HIGH = 900;

const int CUSTOM_LOW = 100;

const int SENSOR1_TIMEOUT = 200;

const int SENSOR2_TIMEOUT = 1900; // the same or a little bigger value should be in python

const int SENSOR2_READ_FREQ = 10;

const int SERIAL_INPUT_DELAY = 50;
