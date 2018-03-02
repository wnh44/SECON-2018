// SECON_Arduino.ino
// Description: This is the current version of the SECON2018 robot.
//              Hopefully it's not broken...

///////////////
// Libraries //
///////////////

#include <Wire.h>
#include <Adafruit_MotorShield.h>


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

// Restart Button (same pin as the start button)
#define RESTART_BUTTON 2


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


//////////////////////
// Course Variables //
//////////////////////

int locations[3] = {0, 0, 0};


///////////////////////////
// Rangefinder Variables //
///////////////////////////

int rangefinder0 = 0;
int rangefinder1 = 0;
int rangefinder2 = 0;
int rangefinder3 = 0;
int rangefinder4 = 0;


///////////////////////////
// Microswitch Variables //
///////////////////////////

int microswitch0 = 0;
int microswitch1 = 0;
int microswitch2 = 0;
int microswitch3 = 0;


/////////////////////
// Motor Variables //
/////////////////////

// Motor Shield Variables
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor0 = AFMS.getMotor(1);
Adafruit_DCMotor *motor1 = AFMS.getMotor(2);
Adafruit_DCMotor *motor2 = AFMS.getMotor(3);
Adafruit_DCMotor *motor3 = AFMS.getMotor(4);

// Command Velocities (0-255)
uint8_t motor0_commandVelocity = 0;
uint8_t motor1_commandVelocity = 0;
uint8_t motor2_commandVelocity = 0;
uint8_t motor3_commandVelocity = 0;

////////////////////////////////
// Pi<->Mega Serial Variables //
////////////////////////////////

char serialMessage[10];


/////////////////////////
// Function Prototypes //
/////////////////////////

void readRangefinders(int wait = 300);


//////////////////////////////////////////////////////////////////////////////////////
// Function: setup()                                                                //
// Description: Initialize variables, serial connections, preheat the oven, blah,   //
//              blah, blah, yakety yak                                              //
//////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Serial.begin(9600);
    Serial2.begin(57600);
    
    // Set Interrupts for Motor Encoders
    //attachInterrupt(5, motor0_encoder_ISR, CHANGE);
    
    // Initialize Microswitch Pins
    pinMode(MICROSWITCH_0, INPUT_PULLUP);
    pinMode(MICROSWITCH_1, INPUT_PULLUP);
    pinMode(MICROSWITCH_2, INPUT_PULLUP);
    pinMode(MICROSWITCH_3, INPUT_PULLUP);
    
    // Initialize Start Button Pin (also initializes restart)
    pinMode(START_BUTTON, INPUT_PULLUP);
    
    // Initialize MotorShield
    AFMS.begin();

    // Initialize motor speed to 0
    motor0->setSpeed(0);
    motor1->setSpeed(0);
    motor2->setSpeed(0);
    motor3->setSpeed(0);  

    // Set initial direction to FORWARD
    motor0->run(FORWARD);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor3->run(FORWARD);

    // Release motors
    motor0->run(RELEASE);
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);

    // Allow time for rangefinders to calibrate
    delay(350);
}


//////////////////////////////////////////////////////////
// Function: loop()                                     //
// Description: Main loop that houses the state machine //
//////////////////////////////////////////////////////////

void loop() {
    Serial.println("D:WAIT_FOR_START");
    
    while(digitalRead(START_BUTTON)) {
        delay(100);
    }
    
    slideBackLeft(127);
}


///////////////////////
// Read Rangefinders //
///////////////////////

void readRangefinders(int wait) {
    digitalWrite(RANGEFINDER_0_RX, HIGH);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, LOW);
    delay(wait);
        
    rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
    Serial2.print('R0:');
    Serial2.println(rangefinder0);

    rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
    Serial2.print('R1:');
    Serial2.println(rangefinder1);

    rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
    Serial2.print('R2:');
    Serial2.println(rangefinder2);

    rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
    Serial2.print('R3:');
    Serial2.println(rangefinder3);

    rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
    Serial2.print('R4:');
    Serial2.println(rangefinder4);
}


////////////////////////
// Read Microswitches //
////////////////////////

void readMicroswitches() {
    microswitch0 = digitalRead(MICROSWITCH_0);
    Serial2.print('U0:');
    Serial2.println(microswitch0);
    
    microswitch1 = digitalRead(MICROSWITCH_1);
    Serial2.print('U1:');
    Serial2.println(microswitch1);
    
    microswitch2 = digitalRead(MICROSWITCH_2);
    Serial2.print('U2:');
    Serial2.println(microswitch2);
    
    microswitch3 = digitalRead(MICROSWITCH_3);
    Serial2.print('U3:');
    Serial2.println(microswitch3);
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

