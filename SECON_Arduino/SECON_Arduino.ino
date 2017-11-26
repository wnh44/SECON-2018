
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
// NOTE: Debug serial port has to be Serial2 or Serial3 if interrupts are used
#define DEBUG_SERIAL3

#if defined(DEBUG_SERIAL1)
    #define DEBUG_BEGIN(x)        Serial1.begin(x)
    #define DEBUG_TIMEOUT(x)      Serial1.setTimeout(x)
    #define DEBUG_PRINT(x)        Serial1.print(x)
    #define DEBUG_PRINT_DEC(x)    Serial1.print(x, DEC)
    #define DEBUG_PRINT_HEX(x)    Serial1.print(x, HEX)
    #define DEBUG_PRINT_0(x)      Serial1.print(x, 0)
    #define DEBUG_PRINT_2(x)      Serial1.print(x, 2)
    #define DEBUG_PRINT_4(x)      Serial1.print(x, 4)
    #define DEBUG_PRINTLN(x)      Serial1.println(x)
    #define DEBUG_PRINTLN_DEC(x)  Serial1.println(x, DEC)
    #define DEBUG_PRINTLN_HEX(x)  Serial1.println(x, HEX)
    #define DEBUG_PRINTLN_0(x)    Serial1.println(x, 0)
    #define DEBUG_PRINTLN_2(x)    Serial1.println(x, 2)
    #define DEBUG_PRINTLN_4(x)    Serial1.println(x, 4)
#elif defined(DEBUG_SERIAL2)
    #define DEBUG_BEGIN(x)        Serial2.begin(x)
    #define DEBUG_TIMEOUT(x)      Serial2.setTimeout(x)
    #define DEBUG_PRINT(x)        Serial2.print(x)
    #define DEBUG_PRINT_DEC(x)    Serial2.print(x, DEC)
    #define DEBUG_PRINT_HEX(x)    Serial2.print(x, HEX)
    #define DEBUG_PRINT_0(x)      Serial2.print(x, 0)
    #define DEBUG_PRINT_2(x)      Serial2.print(x, 2)
    #define DEBUG_PRINT_4(x)      Serial2.print(x, 4)
    #define DEBUG_PRINTLN(x)      Serial2.println(x)
    #define DEBUG_PRINTLN_DEC(x)  Serial2.println(x, DEC)
    #define DEBUG_PRINTLN_HEX(x)  Serial2.println(x, HEX)
    #define DEBUG_PRINTLN_0(x)    Serial2.println(x, 0)
    #define DEBUG_PRINTLN_2(x)    Serial2.println(x, 2)
    #define DEBUG_PRINTLN_4(x)    Serial2.println(x, 4)
#elif defined(DEBUG_SERIAL3)
    #define DEBUG_BEGIN(x)        Serial3.begin(x)
    #define DEBUG_TIMEOUT(x)      Serial3.setTimeout(x)
    #define DEBUG_PRINT(x)        Serial3.print(x)
    #define DEBUG_PRINT_DEC(x)    Serial3.print(x, DEC)
    #define DEBUG_PRINT_HEX(x)    Serial3.print(x, HEX)
    #define DEBUG_PRINT_0(x)      Serial3.print(x, 0)
    #define DEBUG_PRINT_2(x)      Serial3.print(x, 2)
    #define DEBUG_PRINT_4(x)      Serial3.print(x, 4)
    #define DEBUG_PRINTLN(x)      Serial3.println(x)
    #define DEBUG_PRINTLN_DEC(x)  Serial3.println(x, DEC)
    #define DEBUG_PRINTLN_HEX(x)  Serial3.println(x, HEX)
    #define DEBUG_PRINTLN_0(x)    Serial3.println(x, 0)
    #define DEBUG_PRINTLN_2(x)    Serial3.println(x, 2)
    #define DEBUG_PRINTLN_4(x)    Serial3.println(x, 4)
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
    #define DEBUG_PRINTLN_DEC(x)
    #define DEBUG_PRINTLN_HEX(x)
    #define DEBUG_PRINTLN_0(x)
    #define DEBUG_PRINTLN_2(x)
    #define DEBUG_PRINTLN_4(x)
#endif


/////////////////////
// Pin Definitions //
/////////////////////

// Encoder Pins
#define MOTOR_0_ENCODER_A 18            // FIXME: values
#define MOTOR_0_ENCODER_B 22
#define MOTOR_1_ENCODER_A 19
#define MOTOR_1_ENCODER_B 23
#define MOTOR_2_ENCODER_A 20
#define MOTOR_2_ENCODER_B 24
#define MOTOR_3_ENCODER_A 21
#define MOTOR_3_ENCODER_B 25

// Microswitches
#define MICROSWITCH_0 4
#define MICROSWITCH_1 5
#define MICROSWITCH_2 6
#define MICROSWITCH_3 7

// Ultrasonic Rangefinders
#define RANGEFINDER_0 A8
#define RANGEFINDER_1 A9
#define RANGEFINDER_2 A10
#define RANGEFINDER_3 A11
#define RANGEFINDER_4 A12

// Rangefinder0 RX pin
#define RANGEFINDER_0_FX 0

// Start Button
#define START_BUTTON 2


/////////////////////
// Motor Variables //
/////////////////////

// Time Stamps
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


/////////////////////
// FSM Definitions //
/////////////////////

enum states {
    WAIT_FOR_START,
    DECODE_LED,
    TO_STAGE_A,
    STAGE_A,
    FROM_STAGE_A,
    TO_STAGE_B,
    STAGE_B,
    TO_BOOTY,
    RETRIEVE_BOOTY,
    TO_FLAG,
    RAISE_FLAG,
    TO_SHIP,
    TO_STAGE_C,
    STAGE_C
};
states state = WAIT_FOR_START;

int locations[3] = {0, 0, 0};

////////////////////////////////
// Pi<->Mega Serial Variables //
////////////////////////////////

char operation, mode;
int index, quantity, value;
int digitalValue, analogValue;
int pause = 10;
char serialMessage[9];


////////////////////////////
// Motor Shield Variables //
////////////////////////////

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor0 = AFMS.getMotor(1);
Adafruit_DCMotor *motor1 = AFMS.getMotor(2);
Adafruit_DCMotor *motor2 = AFMS.getMotor(3);
Adafruit_DCMotor *motor3 = AFMS.getMotor(4);

// PID if necessary
// PID FL_PID(&vel, &pwm, &cmd_vel, Kp, Ki, Kd, DIRECT);


void setup() {
    Serial.begin(9600);
    Serial.setTimeout(500);
    
    DEBUG_BEGIN(57600);
    DEBUG_TIMEOUT(500);

    pinMode(MOTOR_0_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_0_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_1_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_1_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_2_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_2_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_3_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_3_ENCODER_B, INPUT_PULLUP);
    
    // Set Interrupts for Motor Encoders
    attachInterrupt(5, motor0_encoder_ISR, CHANGE); 
    attachInterrupt(4, motor1_encoder_ISR, CHANGE); 
    attachInterrupt(3, motor2_encoder_ISR, CHANGE); 
    attachInterrupt(2, motor3_encoder_ISR, CHANGE);
    
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

    digitalWrite(RANGEFINDER_0_RX, HIGH);
    delay(35);
    digitalWrite(RANGE_FINDER_0_RX, LOW);
}

// New loop() with FSM implemented in C
void loop() {
    Serial.print(state);
    switch(state) {
        case (WAIT_FOR_START):
            waitForStart();
            break;
        
        case (DECODE_LED):
            decodeLED();
            break;
        
        case (TO_STAGE_A):
            toStageA();
            break;
        
        case (STAGE_A):
            stageA();
            break;
        
        case (FROM_STAGE_A):
            fromStageA();
            break;
        
        case (TO_STAGE_B):
            toStageB();
            break;
        
        case (STAGE_B):
            stageB();
            break;
        
        case (TO_BOOTY):
            toBooty();
            break;
        
        case (RETRIEVE_BOOTY):
            retrieveBooty();
            break;
        
        case (TO_FLAG):
            toFlag();
            break;
        
        case (RAISE_FLAG):
            raiseFlag();
            break;
        
        case (TO_SHIP):
            toShip();
            break;
        
        case (TO_STAGE_C):
            toStageC();
            break;
        
        case (STAGE_C):
            stageC();
            break;
        
        default:
            break;
    }
}


void moveForward(int velocity = 255) {
    motor0_commandVelocity = velocity;
    motor1_commandVelocity = velocity;
    motor2_commandVelocity = velocity;
    motor3_commandVelocity = velocity;
    motor0->run(FORWARD);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor3->run(FORWARD);

    commandMotors();
}

void moveBackward(int velocity = 255) {
    motor0_commandVelocity = velocity;
    motor1_commandVelocity = velocity;
    motor2_commandVelocity = velocity;
    motor3_commandVelocity = velocity;
    motor0->run(BACKWARD);
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
    motor3->run(BACKWARD);

    commandMotors();
}


void moveLeft(int velocity = 255) {
    motor0_commandVelocity = velocity;
    motor1_commandVelocity = velocity;
    motor2_commandVelocity = velocity;
    motor3_commandVelocity = velocity;
    motor0->run(BACKWARD);
    motor1->run(FORWARD);
    motor2->run(BACKWARD);
    motor3->run(FORWARD);

    commandMotors();
}


void moveRight(int velocity = 255) {
    motor0_commandVelocity = velocity;
    motor1_commandVelocity = velocity;
    motor2_commandVelocity = velocity;
    motor3_commandVelocity = velocity;
    motor0->run(FORWARD);
    motor1->run(BACKWARD);
    motor2->run(FORWARD);
    motor3->run(BACKWARD);

    commandMotors();
}


void turnLeft(int velocity = 127) {
    motor1_commandVelocity = velocity;
    motor2_commandVelocity = velocity;
    motor1->run(FORWARD);
    motor2->run(FORWARD);

    commandMotors();
}


void turnRight(int velocity = 127) {
    motor0_commandVelocity = velocity;
    motor3_commandVelocity = velocity;
    motor0->run(FORWARD);
    motor3->run(FORWARD);

    commandMotors();
}

void stopMotors() {
    motor0_commandVelocity = 0;
    motor1_commandVelocity = 0;
    motor2_commandVelocity = 0;
    motor3_commandVelocity = 0;
    motor0->run(RELEASE);
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);

    commandMotors();
}

// FIXE: Comments

void waitForStart() {
    while(!digitalRead(START_BUTTON)) {
        // keep on keeping on
    }
    state = DECODE_LED;
}

void decodeLED() {
    // FIXME: Returns a random number for now
    for(int i = 0; i < 3; i++) {
        locations[i] = random(0, 2);
    }
    Serial.print("Locations: ");
    Serial.print(locations[0]);
    Serial.print(", ");
    Serial.print(locations[1]);
    Serial.print(", ");
    Serial.println(locations[2]);
    
    state = TO_STAGE_A;
}

void toStageA() {
    moveBackward();

    while(digitalRead(MICROSWITCH_0) & digitalRead(MICROSWITCH_1) == 0) {
        Serial.print(digitalRead(MICROSWITCH_0) & digitalRead(MICROSWITCH_1));
        // keep on keeping on
    }
    stopMotors();
    
    if(locations[0] == 0) {
        moveLeft();
    }
    
    state = STAGE_A;
}

void stageA() {
    
    state = FROM_STAGE_A;
}

void fromStageA() {
    
    state = TO_STAGE_B;
}

void toStageB() {
    
    state = STAGE_B;
}

void stageB() {
    
    state = TO_BOOTY;
}

void toBooty() {
    
    state = RETRIEVE_BOOTY;
}

void retrieveBooty() {
    
    state = TO_FLAG;
}

void toFlag() {
    
    state = RAISE_FLAG;
}

void raiseFlag() {
    
    state = RAISE_FLAG;
}

void toShip() {
    
    state = TO_STAGE_C;
}

void toStageC() {
    
    state = STAGE_C;
}

void stageC() {
    
    state = WAIT_FOR_START;
} 

/*
// Old loop() which contained serial communication
void loop() {
    if (Serial.available() > 0) {
        memset(serialMessage, 0, sizeof(serialMessage));
        operation = Serial.read();
        DEBUG_PRINTLN(operation);
        delay(pause); // May be necessary elsewhere
        
        mode = Serial.read();
        DEBUG_PRINTLN(mode);
        delay(pause);
        
        index = Serial.parseInt();
        DEBUG_PRINTLN(index);
        delay(pause);
        sprintf(serialMessage, "%c%c%d", operation, mode, index);
        
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
        DEBUG_PRINTLN(serialMessage);
        
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
        delay(pause);
    }
}
*/


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


