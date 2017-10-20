/*
 *  Serial data is sent in the following manner:
 *
 *  {operation}{mode}{number}{:}{quantity}{:}{value}
 * 
 *  {operation}
 *    M: Set motor speed
 *    P: Set pin mode
 *    R: Read value
 *    W: Write value
 *    
 *  {mode}
 *    A: Analog
 *    D: Digital
 *    I: INPUT
 *    O: OUTPUT
 *    P: INPUT_PULLUP
 *    
 *  {number}
 *    If M: Motor number to set.
 *    If P: N/A
 *    If R: Pin number to read from.
 *    If W: Pin number to write to.
 *    
 *  {quantity}
 *    If M: Number of (sequential) motors to set.
 *    If P: N/A
 *    If R: Number of (sequential) inputs to read.
 *    If W: N/A
 *    
 *  {value}
 *    If M: Motor speed to set.
 *    If P: N/A
 *    If R: N/A
 *    If W: Value to write.
 *  
 *  Examples:
 *    Set Pin Mode:  PI4
 *    Digital Read:  DR7:3
 *    Digital Write: DW4:0
 *    Analog Read:   AR4:0
 *    Analog Write:  AW0:759
 */

///////////////
// Libraries //
///////////////

#include <Wire.h>
#include "utility/Adafruit_MotorShield.h"
//#include "utility/Adafruit_MS_PWMServoDriver.h"


/////////////////////
// Pin Definitions //
/////////////////////

// Encoder Pins
#define MOTOR_0_ENCODER_A 0                                        // FIXME: values
#define MOTOR_0_ENCODER_B 0
#define MOTOR_1_ENCODER_A 0
#define MOTOR_1_ENCODER_B 0
#define MOTOR_2_ENCODER_A 0
#define MOTOR_2_ENCODER_B 0
#define MOTOR_3_ENCODER_A 0
#define MOTOR_3_ENCODER_B 0

// Microswitches
#define MICROSWITCH_0 0
#define MICROSWITCH_1 0
#define MICROSWITCH_2 0
#define MICROSWITCH_3 0

// Ultrasonic Rangefinders
#define RANGEFINDER_0 0
#define RANGEFINDER_1 0
#define RANGEFINDER_2 0
#define RANGEFINDER_3 0
#define RANGEFINDER_4 0


/////////////////////////////
// State Machine Variables //
/////////////////////////////

// States
enum STATE {
    WAIT_FOR_START,
    START,
    RAPE_AND_PILAGE,
    RECIEVE_LED,
    TO_STAGE_A,
    STAGE_A,
    FROM_STAGE_A,
    TO_STAGE_B,
    TO_CENTER,
    TO_BOOTY,
    RETRIEVE_BOOTY,
    TO_FLAG,
    TO_SHIP,
    TO_STAGE_C,
    STAGE_C,
    SAIL_THE_HIGH_SEAS
}


/////////////////////
// Motor Variables //
/////////////////////

// Time Stamps
volatile temp;                                          // FIXME: No clue
volatile int prevTime = 0;
volatile int curTime = 0;

// Encoder Counts
volatile int motor0_encoder = 0;
volatile int motor1_encoder = 0;
volatile int motor2_encoder = 0;
volatile int motor3_encoder = 0;

// Velocity Averages
double motor0_encoderVelocities[5] = {0, 0, 0, 0, 0};
double motor1_encoderVelocities[5] = {0, 0, 0, 0, 0};
double motor2_encoderVelocities[5] = {0, 0, 0, 0, 0};
double motor3_encoderVelocities[5] = {0, 0, 0, 0, 0};

// Motor Velocities (mm/sec)
double motor0_velocity = 0;
double motor1_velocity = 0;
double motor2_velocity = 0;
double motor3_velocity = 0;

// Command Velocities (mm/sec)
double motor0_commandVelocity = 0;
double motor1_commandVelocity = 0;
double motor2_commandVelocity = 0;
double motor3_commandVelocity = 0;

// PWM Commands
double motor0_pwm = 0;
double motor1_pwm = 0;
double motor2_pwm = 0;
double motor3_pwm = 0;

// Motor 0 PID
double motor0_Kp = 1.4;
double motor0_Ki = 6.0;
double motor0_Kd = 0.1;

// Motor 1 PID
double motor1_Kp = 1.4;
double motor1_Ki = 6.0;
double motor1_Kd = 0.1;

// Motor 2 PID
double motor2_Kp = 1.4;
double motor2_Ki = 6.0;
double motor2_Kd = 0.1;

// Motor 3 PID
double motor3_Kp = 1.4;
double motor3_Ki = 6.0;
double motor3_Kd = 0.1;


////////////////////////////////
// Pi<->Mega Serial Variables //
////////////////////////////////

char operation, mode;
int index, quantity, value;
int digitalValue, analogValue;
int pause = 5;


////////////////////////////
// Motor Shield Variables //
////////////////////////////

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *motor0 = AFMS.getMotor(1);
Adafruit_DCMotor *motor1 = AFMS.getMotor(2);
Adafruit_DCMotor *motor2 = AFMS.getMotor(3);
Adafruit_DCMotor *motor3 = AFMS.getMotor(4);

// PID if necessary
// PID FL_PID(&vel, &pwm, &cmd_vel, Kp, Ki, Kd, DIRECT);


void setup() {
    Serial.begin(9600);
    Serial.setTimeout(100);

    pinMode(MOTOR_0_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_0_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_1_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_1_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_2_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_2_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_3_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_3_ENCODER_B, INPUT_PULLUP);

    // Set Interrupts for Motor Encoders
    attachInterrupt(digitalPinToInterrupt(MOTOR_0_ENCODER_A), motor0_encoder_ISR, CHANGE); 
    attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCODER_A), motor1_encoder_ISR, CHANGE); 
    attachInterrupt(digitalPinToInterrupt(MOTOR_2_ENCODER_A), motor2_encoder_ISR, CHANGE); 
    attachInterrupt(digitalPinToInterrupt(MOTOR_3_ENCODER_A), motor3_encoder_ISR, CHANGE);

    // Initialize Microswitch Pins
    pinMode(MICROSWITCH_0, INPUT_PULLUP);
    pinMode(MICROSWITCH_1, INPUT_PULLUP);
    pinMode(MICROSWITCH_2, INPUT_PULLUP);
    pinMode(MICROSWITCH_3, INPUT_PULLUP);
}

void loop() {
    if (Serial.available() > 0) {
        operation = Serial.read();
        delay(pause); // May be necessary elsewhere
        mode = Serial.read();
        index = Serial.parseInt();
        if (Serial.read() == ':') {
            if (operation == 'W') {
                value = Serial.parseInt();
            } else {
                quantity = Serial.parseInt();
            }
            if (operation == 'M') {
                value = Serial.parseInt();
            }
        }
        
        switch(operation) {
            case 'M':
                setMotorSpeedLocal(mode, index, quantity, value);
                break;
      
            case 'P':
                pinModeLocal(index, mode);
                break;
                
            case 'R':
                if (mode == 'D') {
                    digitalReadLocal(index, quantity);
                } else if (mode == 'A') {
                    analogReadLocal(index, quantity);
                } else {
                    break;
                }
                break;
              
            case 'W':
                if (mode == 'D') {
                    digitalWriteLocal(index, value);
                } else if (mode == 'A') {
                    analogWriteLocal(index, value);
                } else {
                    break;
                }
                break;
      
            default:
                break;
        }
    }
}


/////////////////////////////
// ISR for Motor 0 Encoder //
/////////////////////////////

void motor0_encoderA_ISR() {
    // Halts interrupts
    cli();
    
    static volatile int enc;
    
    // Reads the two pins and xors them
    //enc = ((PINE & (1<<PE4))>>4) ^ ((PINH & (1<<PH1))>>1);
    
    switch(enc){
        case (0b1):  // CCW Forward
            motor0_encoder--;
            break;
        
        case (0b0):  // CW Backwards
            motor0_encoder++;
            break;

    // Resumes interrupts
    sei();
    }
}


/////////////////////////////
// ISR for Motor 1 Encoder //
/////////////////////////////

void motor1_encoder_ISR() {
    // Halts interrupts
    cli();
    
    static volatile int enc;
    
    // Reads the two pins and xors them
    //enc = ((PINE & (1<<PE4))>>4) ^ ((PINH & (1<<PH1))>>1);
    
    switch(enc){
        case (0b1):  // CCW Forward
            motor1_encoder--;
            break;
        
        case (0b0):  // CW Backwards
            motor1_encoder++;
            break;

    // Resumes interrupts
    sei();
    }
}


/////////////////////////////
// ISR for Motor 2 Encoder //
/////////////////////////////

void motor2_encoder_ISR() {
    // Halts interrupts
    cli();
    
    static volatile int enc;
    
    // Reads the two pins and xors them
    //enc = ((PINE & (1<<PE4))>>4) ^ ((PINH & (1<<PH1))>>1);
    
    switch(enc){
        case (0b1):  // CCW Forward
            motor2_encoder--;
            break;
        
        case (0b0):  // CW Backwards
            motor2_encoder++;
            break;

    // Resumes interrupts
    sei();
    }
}


/////////////////////////////
// ISR for Motor 3 Encoder //
/////////////////////////////

void motor3_encoder_ISR() {
    // Halts interrupts
    cli();
    
    static volatile int enc;
    
    // Reads the two pins and xors them
    //enc = ((PINE & (1<<PE4))>>4) ^ ((PINH & (1<<PH1))>>1);
    
    switch(enc){
        case (0b1):  // CCW Forward
            motor3_encoder--;
            break;
        
        case (0b0):  // CW Backwards
            motor3_encoder++;
            break;

    // Resumes interrupts
    sei();
    }
}

void pinModeLocal(int pinNumber, char mode) {
    switch(mode){
        case 'I':
            pinMode(pinNumber, INPUT);
            break;
        case 'O':
            pinMode(pinNumber, OUTPUT);
            break;
        case 'P':
            pinMode(pinNumber, INPUT_PULLUP);
            break;
    }
}

void digitalReadLocal(int pinNumber, int quantity) {
    for (int i = 0; i < quantity - 1; i++) {
        digitalValue = digitalRead(pinNumber + i);
        Serial.print(digitalValue);
        Serial.print(':');
    }
    digitalValue = digitalRead(pinNumber + quantity - 1);
    Serial.println(digitalValue);
}

void digitalWriteLocal(int pinNumber, int digitalValue) {
    digitalWrite(pinNumber, digitalValue);
}

void analogReadLocal(int pinNumber, int quantity) {
    for (int i = 0; i < quantity - 1; i++) {
        analogValue = analogRead(pinNumber);
        Serial.print(analogValue);
        Serial.print(':');
    }
    analogValue = analogRead(pinNumber + quantity - 1);
    Serial.println(analogValue);
}

void analogWriteLocal(int pinNumber, int analogValue) {
    analogWrite(pinNumber, analogValue);
}

void setMotorSpeedLocal(char mode, int motorNumber, int quantity, int value) {
    // FIXME
}
