
///////////////
// Libraries //
///////////////

#include <Wire.h>
#include "Adafruit_MotorShield.h"
//#include "utility/Adafruit_MS_PWMServoDriver.h"


/////////////////////////////////////////////
// Defines for serial connection for debug //
/////////////////////////////////////////////

// define desired serial port
#define DEBUG_SERIAL3

#if defined(DEBUG_SERIAL1)
    #define DEBUG_BEGIN(x)      Serial1.begin(x)
    #define DEBUG_TIMEOUT(x)    Serial1.setTimeout(x)
    #define DEBUG_PRINT(x)      Serial1.print(x)
    #define DEBUG_PRINT_DEC(x)  Serial1.print(x, DEC)
    #define DEBUG_PRINT_HEX(x)  Serial1.print(x, HEX)
    #define DEBUG_PRINT_0(x)    Serial1.print(x, 0)
    #define DEBUG_PRINT_2(x)    Serial1.print(x, 2)
    #define DEBUG_PRINT_4(x)    Serial1.print(x, 4)
    #define DEBUG_PRINTLN(x)    Serial1.println(x)
    #define DEBUG_PRINT_DEC(x)  Serial1.println(x, DEC)
    #define DEBUG_PRINT_HEX(x)  Serial1.println(x, HEX)
    #define DEBUG_PRINT_0(x)    Serial1.println(x, 0)
    #define DEBUG_PRINT_2(x)    Serial1.println(x, 2)
    #define DEBUG_PRINT_4(x)    Serial1.println(x, 4)
#elif defined(DEBUG_SERIAL2)
    #define DEBUG_BEGIN(x)      Serial2.begin(x)
    #define DEBUG_TIMEOUT(x)    Serial2.setTimeout(x)
    #define DEBUG_PRINT(x)      Serial2.print(x)
    #define DEBUG_PRINT_DEC(x)  Serial2.print(x, DEC)
    #define DEBUG_PRINT_HEX(x)  Serial2.print(x, HEX)
    #define DEBUG_PRINT_0(x)    Serial2.print(x, 0)
    #define DEBUG_PRINT_2(x)    Serial2.print(x, 2)
    #define DEBUG_PRINT_4(x)    Serial2.print(x, 4)
    #define DEBUG_PRINTLN(x)    Serial2.println(x)
    #define DEBUG_PRINT_DEC(x)  Serial2.println(x, DEC)
    #define DEBUG_PRINT_HEX(x)  Serial2.println(x, HEX)
    #define DEBUG_PRINT_0(x)    Serial2.println(x, 0)
    #define DEBUG_PRINT_2(x)    Serial2.println(x, 2)
    #define DEBUG_PRINT_4(x)    Serial2.println(x, 4)
#elif defined(DEBUG_SERIAL3)
    #define DEBUG_BEGIN(x)      Serial3.begin(x)
    #define DEBUG_TIMEOUT(x)    Serial3.setTimeout(x)
    #define DEBUG_PRINT(x)      Serial3.print(x)
    #define DEBUG_PRINT_DEC(x)  Serial3.print(x, DEC)
    #define DEBUG_PRINT_HEX(x)  Serial3.print(x, HEX)
    #define DEBUG_PRINT_0(x)    Serial3.print(x, 0)
    #define DEBUG_PRINT_2(x)    Serial3.print(x, 2)
    #define DEBUG_PRINT_4(x)    Serial3.print(x, 4)
    #define DEBUG_PRINTLN(x)    Serial3.println(x)
    #define DEBUG_PRINT_DEC(x)  Serial3.println(x, DEC)
    #define DEBUG_PRINT_HEX(x)  Serial3.println(x, HEX)
    #define DEBUG_PRINT_0(x)    Serial3.println(x, 0)
    #define DEBUG_PRINT_2(x)    Serial3.println(x, 2)
    #define DEBUG_PRINT_4(x)    Serial3.println(x, 4)
#else
    #define DEBUG_BEGIN(x)
    #define DEBUG_TIMEOUT(x)
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINT_DEC(x)
    #define DEBUG_PRINT_HEX(x)
    #define DEBUG_PRINT_0(x)
    #define DEBUG_PRINT_2(x)
    #define DEBUG_PRINT_4(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINT_DEC(x)
    #define DEBUG_PRINT_HEX(x)
    #define DEBUG_PRINT_0(x)
    #define DEBUG_PRINT_2(x)
    #define DEBUG_PRINT_4(x)
#endif


/////////////////////
// Pin Definitions //
/////////////////////

// Encoder Pins
#define MOTOR_0_ENCODER_A 0            // FIXME: values
#define MOTOR_0_ENCODER_B 0
#define MOTOR_1_ENCODER_A 0
#define MOTOR_1_ENCODER_B 0
#define MOTOR_2_ENCODER_A 0
#define MOTOR_2_ENCODER_B 0
#define MOTOR_3_ENCODER_A 0
#define MOTOR_3_ENCODER_B 0

// Microswitches
#define MICROSWITCH_0 53
#define MICROSWITCH_1 0
#define MICROSWITCH_2 0
#define MICROSWITCH_3 0

// Ultrasonic Rangefinders
#define RANGEFINDER_0 0
#define RANGEFINDER_1 0
#define RANGEFINDER_2 0
#define RANGEFINDER_3 0
#define RANGEFINDER_4 0


/////////////////////
// Motor Variables //
/////////////////////

// Time Stamps
volatile int temp;                                          // FIXME: No clue
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
uint8_t motor0_commandVelocity = 0;
uint8_t motor1_commandVelocity = 0;
uint8_t motor2_commandVelocity = 0;
uint8_t motor3_commandVelocity = 0;

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
char serialMessage[9];


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
    Serial.begin(57600);
    Serial.setTimeout(100);
    
    DEBUG_BEGIN(115200);
    DEBUG_TIMEOUT(100);

    /*pinMode(MOTOR_0_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_0_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_1_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_1_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_2_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_2_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_3_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_3_ENCODER_B, INPUT_PULLUP);
    // Set Interrupts for Motor Encoders
    attachInterrupt(MOTOR_0_ENCODER_A, motor0_encoder_ISR, CHANGE); 
    attachInterrupt(MOTOR_1_ENCODER_A, motor1_encoder_ISR, CHANGE); 
    attachInterrupt(MOTOR_2_ENCODER_A, motor2_encoder_ISR, CHANGE); 
    attachInterrupt(MOTOR_3_ENCODER_A, motor3_encoder_ISR, CHANGE);*/
    
    // Initialize Microswitch Pins
    pinMode(MICROSWITCH_0, INPUT_PULLUP);
    pinMode(MICROSWITCH_1, INPUT_PULLUP);
    pinMode(MICROSWITCH_2, INPUT_PULLUP);
    pinMode(MICROSWITCH_3, INPUT_PULLUP);
    
    // Initialize MotorShield and Motors
    AFMS.begin();
    motor0->setSpeed(0);
    motor1->setSpeed(0);
    motor2->setSpeed(0);
    motor3->setSpeed(0);    
    motor0->run(FORWARD);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor3->run(FORWARD);
    motor0->run(RELEASE);
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);
}

void loop() {
    if (Serial.available() > 0) {
        memset(serialMessage, 0, sizeof(serialMessage));
        operation = Serial.read();
        delay(pause); // May be necessary elsewhere
        mode = Serial.read();
        index = Serial.parseInt();
        
        sprintf(serialMessage, "%c%c%c", operation, mode, index);
        DEBUG_PRINTLN(serialMessage);
        
        if (Serial.read() == ':') {
            if (operation == 'W') {
                value = Serial.parseInt();
                sprintf(serialMessage, "%s%c%d", serialMessage, ':', value);
            } else {
                quantity = Serial.parseInt();
                sprintf(serialMessage, "%s%c%d", serialMessage, ':', quantity);
            }
            if (Serial.read() == ':') { // THIS WAS CHANGED!!! Hopefully still works.
                value = Serial.parseInt();
                sprintf(serialMessage, "%s%c%d", serialMessage, ':', value);  
            }
        }
        //DEBUG_PRINTLN(sprintf(serialMessage, "Message Received: %s", serialMessage));
        
        switch(operation) {
            case 'F':
                if (mode == 'L') {
                    // FIXME: Will contain call to IR function
                    break;
                } else if (mode == 'C') {
                    commandMotors();
                    break;
                } else {
                    break;
                }
                
            case 'M':
                for(int i = 0; i < quantity; i++) {
                    // Need to verify
                    setMotorCommandVelocity(mode, (index + i) % 4, value);
                }
                break;
      
            case 'P':
                pinModeLocal(index, mode);
                break;
                
            case 'R':
                if (mode == 'D') {
                    digitalReadLocal(index, quantity);
                    break;
                } else if (mode == 'A') {
                    analogReadLocal(index, quantity);
                    break;
                } else {
                    break;
                }
              
            case 'W':
                if (mode == 'D') {
                    digitalWriteLocal(index, value);
                    break;
                } else if (mode == 'A') {
                    analogWriteLocal(index, value);
                    break;
                } else {
                    break;
                }
      
            default:
                break;
        }
    }
}


/////////////////////////////
// ISR for Motor 0 Encoder //
/////////////////////////////

void motor0_encoder_ISR() {
    // Halts interrupts
    cli();
    
    static volatile int enc;
    
    // Reads the two pins and xors them
    //enc = ((PINE & (1<<PE4))>>4) ^ ((PINH & (1<<PH1))>>1);
    
    switch(enc) {
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
    
    switch(enc) {
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
    
    switch(enc) {
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
    
    switch(enc) {
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


////////////////////////////////////
// Serial Communication Functions //
////////////////////////////////////

/*
 *  Serial data is sent and received in the following manner:
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
 *    F: FORWARD
 *    I: INPUT
 *    O: OUTPUT
 *    P: INPUT_PULLUP
 *    R: BACKWARD
 *    
 *  {number}
 *    If M: Motor number to set.
 *    If P: Pin number to set mode of.
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
 *    Set Motor Speed:  MF0:4:255
 *    Set Motor Speed:  MR2:2:60
 */


//////////////////////
// Setting Pin Mode //
//////////////////////

void pinModeLocal(int pinNumber, char mode) {
    switch(mode) {
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


//////////////////
// Digital Read //
//////////////////

void digitalReadLocal(int pinNumber, int quantity) {
    for (int i = 0; i < quantity - 1; i++) {
        digitalValue = digitalRead(pinNumber + i);
        Serial.print(digitalValue);
        Serial.print(':');
    }
    digitalValue = digitalRead(pinNumber + quantity - 1);
    Serial.println(digitalValue);
}


///////////////////
// Digital Write //
///////////////////

void digitalWriteLocal(int pinNumber, int digitalValue) {
    digitalWrite(pinNumber, digitalValue);
}


/////////////////
// Analog Read //
/////////////////

void analogReadLocal(int pinNumber, int quantity) {
    for (int i = 0; i < quantity - 1; i++) {
        analogValue = analogRead(pinNumber + i);
        Serial.print(analogValue);
        Serial.print(':');
    }
    analogValue = analogRead(pinNumber + quantity - 1);
    Serial.println(analogValue);
}


//////////////////
// Analog Write //
//////////////////

void analogWriteLocal(int pinNumber, int analogValue) {
    analogWrite(pinNumber, analogValue);
}


//////////////////////////////////
// Set Motor Command Velocities //
//////////////////////////////////

void setMotorCommandVelocity(char mode, int motorNumber, uint8_t value) {
    switch(motorNumber) {
        case 0:
            if(mode == 'R') motor0->run(BACKWARD);
            else motor0->run(FORWARD);
            motor0_commandVelocity = value;
        case 1:
            if(mode == 'R') motor1->run(BACKWARD);
            else motor1->run(FORWARD);
            motor1_commandVelocity = value;
        case 2:
            if(mode == 'R') motor2->run(BACKWARD);
            else motor2->run(FORWARD);
            motor2_commandVelocity = value;
        case 3:
            if(mode == 'R') motor3->run(BACKWARD);
            else motor3->run(FORWARD);
            motor3_commandVelocity = value;
    }
}


////////////////////
// Command Motors //
////////////////////

void commandMotors() {
    motor0->setSpeed(motor0_commandVelocity);
    motor1->setSpeed(motor1_commandVelocity);
    motor2->setSpeed(motor2_commandVelocity);
    motor3->setSpeed(motor3_commandVelocity);
}
