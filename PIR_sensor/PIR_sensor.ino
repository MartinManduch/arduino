
const byte PIR_SENSOR_PIN = 12;
const byte BUZZER_PIN = 13;
const byte RELAY_SWITCH_PIN = 11;
const byte CALIBRATION_TIME_SEC = 10;

const int MOTION_DETECTION_DELAY_MSEC = 300;

const byte NUMBER_OF_BUZZER_BEEPS = 5;
const int DELAY_BETWEEN_BEEPS_MSEC = 1000; //msec
const int BUZZER_BEEP_DURATION_MSEC = 20; //msec
const unsigned long ALARM_DURATION_MSEC = 120000;

enum states{MONITORING, BUZZER_SIGNALLING, SIREN_ALARM_ON};

enum states state;


// 3 measurements of HIGH signal shows motion detected, normally one would be enough, 3 are for theoretical noise suppression
// by analogread I can see, the signal is at least for 2 seconds
// we'll measure 3 times with delay 300ms, to detect movement
bool isMotionDetected(){
    Serial.println("start isMotionDected");
    bool result = true;
    int i = 0;
    while (i < 3){
        if(digitalRead(PIR_SENSOR_PIN) == LOW){
            result = false;
            Serial.println("isMotionDected false");
        }
        delay(MOTION_DETECTION_DELAY_MSEC);
        i++;
    }
    Serial.println("isMotionDected true");
    return result;
}

void signalByBuzzer(){
    Serial.println("start signalByBuzzer");
    for (int i = 0; i < NUMBER_OF_BUZZER_BEEPS; i++){
        digitalWrite(BUZZER_PIN, HIGH);
        Serial.println("buzzer active");
        delay(BUZZER_BEEP_DURATION_MSEC);
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("buzzer passive");
        delay(DELAY_BETWEEN_BEEPS_MSEC);
    }
    
}

void makeNoiseBySiren(){
  Serial.println("siren started ");
    digitalWrite(RELAY_SWITCH_PIN, HIGH);
    delay(ALARM_DURATION_MSEC);
    digitalWrite(RELAY_SWITCH_PIN, LOW);
    Serial.println("siren stopped");
}

void reset(){
    digitalWrite(RELAY_SWITCH_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    state = MONITORING;
}


void setup(){
    Serial.begin(9600);
    pinMode(PIR_SENSOR_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(RELAY_SWITCH_PIN, OUTPUT);
    digitalWrite(PIR_SENSOR_PIN, LOW);
    
    //give the sensor some time to calibrate
    Serial.print("calibrating sensor ");
    delay(CALIBRATION_TIME_SEC * 1000);
    Serial.println("SENSOR ACTIVE");    
    
    reset();
}

void loop(){
    
    if(digitalRead(PIR_SENSOR_PIN) == HIGH){
        
        Serial.println("HIGH PIR");
        if (isMotionDetected()){
            state = BUZZER_SIGNALLING;
            signalByBuzzer();
            state = SIREN_ALARM_ON;
            makeNoiseBySiren();
            reset();
            delay(1000);
        }
        
    }
    
    delay(MOTION_DETECTION_DELAY_MSEC);
}
