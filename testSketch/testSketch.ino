
///////////////
// Libraries //
///////////////

#include <Wire.h>
#include "Adafruit_MotorShield.h"
//#include "utility/Adafruit_MS_PWMServoDriver.h"


/////////////////////
// Pin Definitions //
/////////////////////

// Encoder Pins
#define MOTOR_0_ENCODER_A 18
#define MOTOR_0_ENCODER_B 22
#define MOTOR_1_ENCODER_A 19
#define MOTOR_2_ENCODER_B 23
#define MOTOR_3_ENCODER_A 20
#define MOTOR_3_ENCODER_B 24
#define MOTOR_4_ENCODER_A 21
#define MOTOR_4_ENCODER_B 25

#define MICROSWITCH_0 7
#define MICROSWITCH_1 6
#define MICROSWITCH_2 5
#define MICROSWITCH_3 4


/////////////////////
// Motor Variables //
/////////////////////

// Time Stamps
volatile int prevTime = 0;
volatile int curTime = 0;

// Encoder Counts
volatile int motor0_encoder = 0;

// Velocity Averages
double motor0_encoderVelocities[5] = {0, 0, 0, 0, 0};

// Motor Velocities (mm/sec)
double motor0_velocity = 0;

// Command Velocities (mm/sec)
uint8_t motor0_commandVelocity = 0;


////////////////////////////
// Motor Shield Variables //
////////////////////////////

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor0 = AFMS.getMotor(1);
Adafruit_DCMotor *motor1 = AFMS.getMotor(2);
Adafruit_DCMotor *motor2 = AFMS.getMotor(3);
Adafruit_DCMotor *motor3 = AFMS.getMotor(4);

void setup() {
    Serial.begin(57600);
    Serial.setTimeout(100);

    pinMode(MOTOR_0_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_0_ENCODER_B, INPUT_PULLUP);
    
    pinMode(MICROSWITCH_0, INPUT_PULLUP);
    pinMode(MICROSWITCH_1, INPUT_PULLUP);
    pinMode(MICROSWITCH_2, INPUT_PULLUP);
    pinMode(MICROSWITCH_3, INPUT_PULLUP);
    
    // Set Interrupts for motor encoders
    //attachInterrupt(digitalPinToInterrupt(2), motor0_encoder_ISR, CHANGE);
    
    // Initialize MotorShield and Motors
    AFMS.begin();
    motor0->setSpeed(127);
    motor0->run(FORWARD);
    motor1->setSpeed(127);
    motor1->run(FORWARD);
    motor2->setSpeed(127);
    motor2->run(FORWARD);
    motor3->setSpeed(127);
    motor3->run(FORWARD);
    
    /*motor0->setSpeed(0);
    motor1->setSpeed(0);
    motor2->setSpeed(0);
    motor3->setSpeed(0);*/
}

void loop() {
    Serial.print(digitalRead(MICROSWITCH_0));
    Serial.print(digitalRead(MICROSWITCH_1));
    Serial.print(digitalRead(MICROSWITCH_2));
    Serial.println(digitalRead(MICROSWITCH_3));
    delay(100);
}


/////////////////////////////
// ISR for Motor 0 Encoder //
/////////////////////////////

void motor0_encoder_ISR() {
    // Halts interrupts
    //cli();
    
    //static volatile int enc;
    
    // Reads the two pins and xors them
    //enc = ((PINE & (1<<PE4))>>4) ^ ((PINH & (1<<PH1))>>1);
    
    /*switch(enc) {
        case (0b1):  // CCW Forward
            motor0_encoder--;
            break;
        
        case (0b0):  // CW Backwards
            motor0_encoder++;
            break;*/
    curTime = millis();
    Serial.println(curTime - prevTime);
    prevTime = curTime;
    // Resumes interrupts
    //sei();
}

