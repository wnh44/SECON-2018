
///////////////
// Libraries //
///////////////

#include <Wire.h>
#include "Adafruit_MotorShield.h"
//#include "utility/Adafruit_MS_PWMServoDriver.h"


/////////////////////
// Pin Definitions //
/////////////////////

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
    
    // Initialize Microswitch Pins
    pinMode(MICROSWITCH_0, INPUT_PULLUP);
    pinMode(MICROSWITCH_1, INPUT_PULLUP);
    pinMode(MICROSWITCH_2, INPUT_PULLUP);
    pinMode(MICROSWITCH_3, INPUT_PULLUP);
    pinMode(START_BUTTON, INPUT_PULLUP);
    
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

    pinMode(RANGEFINDER_0_RX, OUTPUT);
    
    digitalWrite(RANGEFINDER_0_RX, 1);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, 0);
    
    delay(300);
}

void loop() {
    moveForward(255);
    //delay(10000); // Probably will need.
    
    while((analogRead(RANGEFINDER_4) - 3) / 2 + 3 >= 17) {
        int rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
        int rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
        int rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
        int rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
        
        int leftSum = rangefinder0 + rangefinder3;
        int rightSum = rangefinder1 + rangefinder2;
        
        Serial.print(rangefinder0); Serial.print(" ");
        Serial.print(rangefinder1); Serial.print(" ");
        Serial.print(rangefinder2); Serial.print(" ");
        Serial.print(rangefinder3); Serial.print(" ");
        Serial.print(leftSum); Serial.print(" ");
        Serial.println(rightSum);
        
        while(leftSum - rightSum >= 6) {
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(300);
            
            moveLeft(255);
            
            rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
            rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
            rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
            rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
            
            leftSum = rangefinder0 + rangefinder3;
            rightSum = rangefinder1 + rangefinder2;
        }
        
        while(rightSum - leftSum >= 6) {    
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(300);
            
            moveRight(255);
            
            rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
            rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
            rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
            rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
            
            leftSum = rangefinder0 + rangefinder3;
            rightSum = rangefinder1 + rangefinder2;
        }
            
        
        if(rangefinder0 + rangefinder2 > rangefinder3 + rangefinder1) {
            turnLeft(255);
        } else if(rangefinder0 + rangefinder2 < rangefinder3 + rangefinder1) {
            turnRight(255);
        }
            
        digitalWrite(RANGEFINDER_0_RX, 1);
        delay(35);
        digitalWrite(RANGEFINDER_0_RX, 0);
        moveForward(255);
        delay(300);
    }
    stopRobot();
    
    while(1) {
        delay(1000);
    }
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
    motor0_commandVelocity = 0;
    motor1_commandVelocity = velocity;
    motor2_commandVelocity = velocity;
    motor3_commandVelocity = 0;
    motor1->run(FORWARD);
    motor2->run(FORWARD);

    commandMotors();
}


//////////////////////
// Turn Robot Right //
//////////////////////

void turnRight(int velocity) {
    motor0_commandVelocity = velocity;
    motor1_commandVelocity = 0;
    motor2_commandVelocity = 0;
    motor3_commandVelocity = velocity;
    motor0->run(FORWARD);
    motor3->run(FORWARD);

    commandMotors();
}


///////////////////////////////////
// Slide Robot Left and Backward //
///////////////////////////////////

void slideBackLeft(int velocity) {
    motor0_commandVelocity = velocity;
    motor1_commandVelocity = velocity / 2;
    motor2_commandVelocity = velocity;
    motor3_commandVelocity = velocity / 2;
    motor0->run(BACKWARD);
    motor1->run(FORWARD);
    motor2->run(BACKWARD);
    motor3->run(FORWARD);

    commandMotors();
}


////////////////////////////////////
// Slide Robot Right and Backward //
////////////////////////////////////

void slideBackRight(int velocity) {
    motor0_commandVelocity = velocity / 2;
    motor1_commandVelocity = velocity;
    motor2_commandVelocity = velocity / 2;
    motor3_commandVelocity = velocity;
    motor0->run(FORWARD);
    motor1->run(BACKWARD);
    motor2->run(FORWARD);
    motor3->run(BACKWARD);

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

