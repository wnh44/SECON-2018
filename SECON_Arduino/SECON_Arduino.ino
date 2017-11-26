
///////////////
// Libraries //
///////////////

#include <Wire.h>
#include "Adafruit_MotorShield.h"


/////////////////////
// Pin Definitions //
/////////////////////

// Encoder Pins
#define MOTOR_0_ENCODER_A 18
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
#define RANGEFINDER_0_RX 24

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
    digitalWrite(RANGEFINDER_0_RX, LOW);
}

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
// FIXE: Comments

void waitForStart() {
    while(digitalRead(START_BUTTON)) {
        Serial.println("not yet");
        delay(100);
    }
    state = DECODE_LED;
}


////////////////
// Decode LED //
////////////////

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


/////////////////////////
// Navigate to stage A //
/////////////////////////

void toStageA() {
    moveBackward(255);

    while(!digitalRead(MICROSWITCH_2) || !digitalRead(MICROSWITCH_3)) {
        if(digitalRead(MICROSWITCH_2)) {
            turnLeft(127);
        } else if(digitalRead(MICROSWITCH_3)) {
            turnRight(127);
        } else {
            moveBackward(255);
        }
    }

    // Probably isn't necessary
    stopRobot();

    // Stage A location on North side of course
    if(locations[0] == 0) {    
        // Move left towards Stage A    
        while((analogRead(RANGEFINDER_1) - 3) / 2 + 3 < 33) {
            if(!digitalRead(MICROSWITCH_3)) {
                slideBackLeft(255);
            } else {
                moveLeft(255);
            }
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            
            delay(300);
        }
    
        // Slow down when ~5 in away
        while((analogRead(RANGEFINDER_1) - 3) / 2 + 3 < 38) {
            if(!digitalRead(MICROSWITCH_2) && !digitalRead(MICROSWITCH_3)) {
                moveBackward(127);
            } else {
                moveLeft(127);
            }
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            
            delay(300);
        }
    }
    
    // Stage A location on North side of course
    else {
        // Move right towards Stage A
        moveRight(255);
        while(analogRead(RANGEFINDER_0) < 33) {
            // Keep on keeping on
        }

        // Slow down when ~5 in away
        moveRight(127);
        while(analogRead(RANGEFINDER_0) < 38) {
            // Keep on keeping on
        }
    }

    stopRobot();
    
    state = STAGE_A;
}

void stageA() {
    // FIXME: Stage A implementation
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


////////////////////////
// Move Robot Forward //
////////////////////////

void moveForward(int velocity) {
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


/////////////////////////
// Move Robot Backward //
/////////////////////////

void moveBackward(int velocity) {
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


/////////////////////
// Move Robot Left //
/////////////////////

void moveLeft(int velocity) {
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


//////////////////////
// Move Robot Right //
//////////////////////

void moveRight(int velocity) {
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


/////////////////////
// Turn Robot Left //
/////////////////////

void turnLeft(int velocity) {
    motor1_commandVelocity = velocity;
    motor2_commandVelocity = velocity;
    motor1->run(FORWARD);
    motor2->run(FORWARD);

    commandMotors();
}


//////////////////////
// Turn Robot Right //
//////////////////////

void turnRight(int velocity) {
    motor0_commandVelocity = velocity;
    motor3_commandVelocity = velocity;
    motor0->run(FORWARD);
    motor3->run(FORWARD);

    commandMotors();
}


////////////////
// Stop Robot //
////////////////

void stopRobot() {
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

