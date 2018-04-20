// testSketch.ino
// Description: Used to test individual functions and such.

///////////////
// Libraries //
///////////////

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


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

// Rangefinder RX
#define RANGEFINDER_0_RX 23
#define RANGEFINDER_1_RX 25
#define RANGEFINDER_2_RX 27
#define RANGEFINDER_3_RX 29
#define RANGEFINDER_4_RX 31

// Start Button
#define START_BUTTON 1

// Servos
#define SERVO_FLAG 11
#define SERVO_STAGE_B0 9
#define SERVO_STAGE_B1 10

// OLED Screen
#define OLED_RESET 3


/////////////////////
// FSM Definitions //
/////////////////////

enum states {
    WAIT_FOR_START,
    DECODE_LED,
    TO_STAGE_A0,
    TO_STAGE_A1,
    STAGE_A,
    FROM_STAGE_A0,
    FROM_STAGE_A1,
    TO_CENTER_OF_COURSE,
    TO_STAGE_B0,
    TO_STAGE_B1,
    STAGE_B0,
    STAGE_B1,
    TO_BOOTY0,
    TO_BOOTY1,
    RETRIEVE_BOOTY,
    TO_FLAG,
    RAISE_FLAG,
    TO_SHIP,
    TO_STAGE_C0,
    TO_STAGE_C1,
    STAGE_C
};
states state = WAIT_FOR_START;


//////////////////////
// Course Variables //
//////////////////////

int locations[3] = {0, 0, 0};

int servoB0_raised = 10;
int servoB0_pressed = 140;

int servoB1_raised = 180;
int servoB1_pressed = 40;


///////////////////////////N
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

// Bottom Motor Shield Variables
Adafruit_MotorShield AFMS_bottom = Adafruit_MotorShield(0x60);
Adafruit_DCMotor *motor0 = AFMS_bottom.getMotor(1);
Adafruit_DCMotor *motor1 = AFMS_bottom.getMotor(2);
Adafruit_DCMotor *motor2 = AFMS_bottom.getMotor(3);
Adafruit_DCMotor *motor3 = AFMS_bottom.getMotor(4);

// Command Velocities (0-255)
uint8_t motor0_commandVelocity = 0;
uint8_t motor1_commandVelocity = 0;
uint8_t motor2_commandVelocity = 0;
uint8_t motor3_commandVelocity = 0;

// Top Motor Shield Variables
Adafruit_MotorShield AFMS_top = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *motorBooty = AFMS_top.getMotor(1);
Adafruit_StepperMotor *motorStepper = AFMS_top.getStepper(200, 2);

// Servo Motors
Servo servoFlag;
Servo servoStageB0;
Servo servoStageB1;

////////////////////
// OLED Variables //
////////////////////

Adafruit_SSD1306 display(OLED_RESET);
#define ZERO 48
#define ONE 49


////////////////////////////////
// Pi<->Mega Serial Variables //
////////////////////////////////

char serialMessage[10];
int temp = 0;


/////////////////////////
// Function Prototypes //
/////////////////////////

//void readRangefinders(int number = 5, int wait = 300);


//////////////////////////////////////////////////////////////////////////////////////
// Function: setup()                                                                //
// Description: Initialize variables, serial connections, preheat the oven, blah,   //
//              blah, blah, yakety yak                                              //
//////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Serial.begin(9600);
    Serial2.begin(57600);
    
    // Initialize Microswitch Pins
    pinMode(MICROSWITCH_0, INPUT_PULLUP);
    pinMode(MICROSWITCH_1, INPUT_PULLUP);
    pinMode(MICROSWITCH_2, INPUT_PULLUP);
    pinMode(MICROSWITCH_3, INPUT_PULLUP);
    
    // Initialize Start Button Pin (also initializes restart)
    pinMode(START_BUTTON, INPUT_PULLUP);
    
    // Initialize OLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();
    delay(1000);
    display.clearDisplay();
    
    // Initialize MotorShields
    AFMS_bottom.begin();
    AFMS_top.begin();

    // Initialize motor speeds
    motor0->setSpeed(0);
    motor1->setSpeed(0);
    motor2->setSpeed(0);
    motor3->setSpeed(0);
    motorBooty->setSpeed(0);
    motorStepper->setSpeed(50);

    // Set initial direction to FORWARD
    motor0->run(FORWARD);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor3->run(FORWARD);
    motorBooty->run(FORWARD);

    // Release motors
    motor0->run(RELEASE);
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);
    motorBooty->run(RELEASE);
    
    // Attach servos to pins and initialize
    servoFlag.attach(SERVO_FLAG);
    servoFlag.writeMicroseconds(1500);
    
    servoStageB0.attach(SERVO_STAGE_B0);
    servoStageB0.write(servoB0_raised);
    
    servoStageB1.attach(SERVO_STAGE_B1);
    servoStageB1.write(servoB1_raised);

    // Initial Readings
    readRangefinders(5, 300);
    readMicroswitches();
    
    ////////////////////////////////
    // Adjustment for Booty Motor //
    ////////////////////////////////
    
    // This tightens/loosens the booty motor on Arduino reset
    
    /* Comment this line to enable
    
    // Tighten cable
    motorBooty->run(BACKWARD);
    
    // Loosen Cable
    //motorBooty->run(FORWARD);
    
    motorBooty->setSpeed(255);
    delay(100);
    motorBooty->run(RELEASE);
    motorBooty->setSpeed(0);
    // */
    
    Serial.println("Setup complete.");
    Serial2.println("D: Setup complete.");
}


//////////////////////////////////////////////////////////
// Function: loop()                                     //
// Description: Main loop that houses the state machine //
//////////////////////////////////////////////////////////

void loop() {
    Serial2.println("D:LOOP");
    
    waitForStart();
    
    decodeLED();
    if(locations[0] == 0) {
        toStageA0();
        fromStageA0();
    } else if(locations[0] == 1) {
        toStageA1();
        fromStageA1();
    }
    toCenterOfCourse();
    
    if(locations[1] == 0) {
        toStageB0();
        stageB0();
        toBooty0();
    } else if(locations[1] == 1) {
        toStageB1();
        stageB1();
        toBooty1();
    } 
    retrieveBooty();
    toFlag();
    raiseFlag();
    toShip();
    
    if(locations[2] == 0) {
        toStageC0();
    } else if(locations[2] == 1) {
        toStageC1();
    } 
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: waitForStart()                                                         //
// Description: Waits at the statring square for the start button to be pressed.    //
//              This will be updated to wait for PWM signal from the IRLED.         //
//////////////////////////////////////////////////////////////////////////////////////

void waitForStart() {
    Serial2.println("D:WAIT_FOR_START");
    
    // Insert code for positioning signal
    
    while(digitalRead(START_BUTTON)) {
        readRangefinders(5, 300);
        readMicroswitches();
    }
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: decodeLED()                                                            //
// Description: Decodes the LED, prints results on LCD, and assigns locations       //
//////////////////////////////////////////////////////////////////////////////////////

void decodeLED() {
    Serial2.println("D:DECODE_LED");
    
    // FIXME: Returns a random number for now
    randomSeed(analogRead(A14));
    for(int i = 0; i < 3; i++) {
        locations[i] = random(0, 2);
    }
    
    //Print to GUI
    Serial2.print("D:Locations - {");
    Serial2.print(locations[0]);
    Serial2.print(", ");
    Serial2.print(locations[1]);
    Serial2.print(", ");
    Serial2.print(locations[2]);
    Serial2.println("}");
    
    //Print to display
    display.setTextSize(4);
    display.setTextColor(WHITE);
    display.setCursor(32, 1);
    display.clearDisplay();
    
    if(locations[0] == 0) display.write(ZERO);
    else if(locations[0] == 1) display.write(ONE);
    
    if(locations[1] == 0) display.write(ZERO);
    else if(locations[1] == 1) display.write(ONE);
    
    if(locations[2] == 0) display.write(ZERO);
    else if(locations[2] == 1) display.write(ONE);
    
    display.display();
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: toStageA0()                                                            //
// Description: Navigates to left stage A                                           //
//////////////////////////////////////////////////////////////////////////////////////

void toStageA0() {
    Serial2.println("D:TO_STAGE_A0");
    readMicroswitches();
    
    moveBackward(255);
    
    // Move to back wall
    while(!microswitch2 || !microswitch3) {
        if(microswitch2) {
            turnLeft(127);
        } else if(microswitch3) {
            turnRight(127);
        } else {
            moveBackward(255);
        }
        readMicroswitches();
    }
    
    Serial2.println("D:To Stage A0");
    readRangefinders(1, 50);
    
    // Move left towards Stage A. If leading microswitch is deactivated, robot
    // slides until contact is reestablished
    while(rangefinder1 < 33) {
        if(!microswitch3) {
            Serial2.println("D:slide");
            slideBackLeft(255);
        } else {
            Serial2.println("D:move");
            moveLeft(255);
        }

        readRangefinders(1, 50);
        readMicroswitches();
    }

    // Slow down when ~2 in away while maintaining contact with back wall
    while(rangefinder1 < 35) {
        Serial2.println("D:close");
        while(!microswitch2 || !microswitch3) {
            moveBackward(127);
            readMicroswitches();
        }
        moveLeft(127);
        
        readRangefinders(1, 50);
        readMicroswitches();
    }

    // Straighten up on back wall
    moveBackward(255);
    Serial2.println("D:To back wall");
    while(!microswitch2 || !microswitch3) {
        if(microswitch2) {
            turnLeft(127);
        } else if(microswitch3) {
            turnRight(127);
        } else {
            moveBackward(255);
        }
        readMicroswitches();
    }
    moveLeft(255);
    
    // Press the shit out of that button
    delay(200);
    
    stopRobot();
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: toStageA1()                                                            //
// Description: Navigates to right stage A                                          //
//////////////////////////////////////////////////////////////////////////////////////

void toStageA1() {
    Serial2.println("D:TO_STAGE_A");
    readMicroswitches();
    
    moveBackward(255);
    
    // Move to back wall
    while(!microswitch2 || !microswitch3) {
        if(microswitch2) {
            turnLeft(127);
        } else if(microswitch3) {
            turnRight(127);
        } else {
            moveBackward(255);
        }
        readMicroswitches();
    }
    
    Serial2.println("D:To Stage A1");
    readRangefinders(0, 50);
    
    // Move right towards Stage A. If leading microswitch is deactivated, robot
    // slides until contact is reestablished
    while(rangefinder0 < 33) {
        if(!microswitch2) {
            slideBackRight(255);
        } else {
            moveRight(255); 
        }
        
        readRangefinders(0, 50);
        readMicroswitches();
    }

    // Slow down when ~2 in away while maintaining contact with back wall
    while(rangefinder0 < 35) {
        Serial2.println("D:close");
        while(!microswitch2 || !microswitch3) {
            moveBackward(127);
            readMicroswitches();
        }
        moveRight(127);
        
        readRangefinders(0, 50);
        readMicroswitches();
    }

    // Straighten up on back wall
    
    moveBackward(255);    
    Serial2.println("D:To back wall");
    
    while(!microswitch2 || !microswitch3) {
        if(microswitch2) {
            turnLeft(127);
        } else if(microswitch3) {
            turnRight(127);
        } else {
            moveBackward(255);
        }
        readMicroswitches();
    }
    moveRight(255);
    
    // Press the shit out of that button
    delay(200);
    
    stopRobot();
}



//////////////////////////////////////////////////////////////////////////////////////
// Function: stageA()                                                               //
// Description: Activates stage A. This probably won"t be necessary.                //
//////////////////////////////////////////////////////////////////////////////////////

void stageA() {
    Serial2.println("D:STAGE_A");
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: fromStageA0()                                                          //
// Description: Navigates from left stage A                                         //
//////////////////////////////////////////////////////////////////////////////////////

void fromStageA0() {
    Serial2.println("D:FROM_STAGE_A0");

    // Move right towards center
    while(rangefinder1 - rangefinder0 >= 3) {
        if(!microswitch2) {
            slideBackRight(255);
        } else {
            moveRight(255);
        }
        
        readRangefinders(0, 20);
        readRangefinders(1, 50);
        
        rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
        Serial2.print("R0:");
        Serial2.println(rangefinder0);
        
        readMicroswitches();
    }

    // Slow down when ~2 in away
    while(rangefinder1 != rangefinder0) {
        while(!microswitch2 || !microswitch3) {
            moveBackward(127);
            readMicroswitches();
        }
        moveRight(32);
        
        readRangefinders(0, 20);
        readRangefinders(1, 50);
        readMicroswitches();
    }
    
    // For now
    stopRobot();
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: fromStageA1()                                                          //
// Description: Navigates from right stage A                                        //
//////////////////////////////////////////////////////////////////////////////////////

void fromStageA1() {
    Serial2.println("D:FROM_STAGE_A1");
   
    // Move left towards center
    while(rangefinder0 - rangefinder1 >= 3) {
        if(!microswitch3) {
            slideBackLeft(255);
        } else {
            moveLeft(255);
        }
        
        readRangefinders(0, 20);
        readRangefinders(1, 50);
        readMicroswitches();
    }

    // Slow down when ~2 in away
    while(rangefinder0 != rangefinder1) {
        while(!microswitch2 || !microswitch3) {
            moveBackward(127);
            readMicroswitches();
        }
        moveLeft(32);
         
        readRangefinders(0, 20);
        readRangefinders(1, 50);
        readMicroswitches();
    }
    
    // For now
    stopRobot();
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: toCenterofCourse()                                                     //
// Description: Navigates to center of course                                       //
//////////////////////////////////////////////////////////////////////////////////////

void toCenterOfCourse() {
    Serial2.println("D:TO_CENTER_OF_COURSE");
    
    moveForward(255);
    delay(4700); // Probably necessary
    readRangefinders(5, 250);
    
    // Navigate towards chest
    while(rangefinder4 >= 17) {        
        int leftSum = rangefinder0 + rangefinder3;
        int rightSum = rangefinder1 + rangefinder2;
        
        while((leftSum - rightSum) >= 4) {
            // Correct robot to left while waiting on sensors (might as well multitask)
            moveLeft(255);
            readRangefinders(5, 100);
            turnRight(155);
            delay(150);
            moveForward(255);

            rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
            Serial2.print("R2:");
            Serial2.println(rangefinder2);
        
            rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
            Serial2.print("R3:");
            Serial2.println(rangefinder3);
        
            rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
            Serial2.print("R4:");
            Serial2.println(rangefinder4);

            // Update sums and calculate again
            leftSum = rangefinder0 + rangefinder3;
            rightSum = rangefinder1 + rangefinder2;
        }
        
        while((rightSum - leftSum) >= 4) {
            // Correct robot to right while waiting on sensors (nobody likes timewasters)
            moveRight(255);
            readRangefinders(5, 100);
            moveForward(255);
            delay(150);

            rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
            Serial2.print("R2:");
            Serial2.println(rangefinder2);
        
            rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
            Serial2.print("R3:");
            Serial2.println(rangefinder3);
        
            rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
            Serial2.print("R4:");
            Serial2.println(rangefinder4);

            // Update sums and calculate again
            leftSum = rangefinder0 + rangefinder3;
            rightSum = rangefinder1 + rangefinder2;
        }
            
        if((rangefinder0 + rangefinder2) > (rangefinder3 + rangefinder1)) {
            turnLeft(255);
        } else if((rangefinder0 + rangefinder2) < (rangefinder3 + rangefinder1)) {
            turnRight(255);
        }
        
        // Turn robot left or right while waiting on sensors (you know the drill)
        
        delay(100);
        moveForward(255);
        readRangefinders(5, 250);
    }
    
    while(rangefinder4 <= 17) {
        Serial2.print("D:here");
        moveForward(255);
        readRangefinders(4, 50);
    }
    
    while(rangefinder4 > 23) {
        Serial2.print("D:there");          
        moveForward(127);
        readRangefinders(4, 50);
    }
    
    stopRobot();
    
    // This is a tweakable variable to ensure robot is in the center
    int enough = 1;

    // Straighten up in the center
    while((abs((rangefinder0 + rangefinder2) - (rangefinder3 + rangefinder1)) <= 1) || (enough > 0)) {
        Serial2.println("D:centering");
        
        // Decrements enough variable if conditions are favorable
        if(abs((rangefinder0 + rangefinder2) - (rangefinder3 + rangefinder1)) <= 1) {
            enough--;
        }

        // Turns robot if needed
        if((rangefinder0 + rangefinder2) > (rangefinder3 + rangefinder1)) {
            turnLeft(63);
        } else if((rangefinder0 + rangefinder2) < (rangefinder3 + rangefinder1)) {
            turnRight(63);
        }

        // Always waiting on sensors...
        delay(50);
        stopRobot();
        readRangefinders(5, 250);
    }
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: toStageB0()                                                            //
// Description: Navigates to left stage B                                           //
//////////////////////////////////////////////////////////////////////////////////////

void toStageB0() {
    Serial2.println("D:TO_STAGE_B0");

    // Move left towards Stage B (0)
    moveLeft(255);
    readMicroswitches();
    
    moveLeft(255);
    delay(2000);
    
    boolean rangefinder1_good = false;
    boolean rangefinder2_good = false;
    
    // Move towards Stage B
    while(!rangefinder1_good || !rangefinder2_good) {
        if(rangefinder1 >= 33) rangefinder1_good = true;
        if(rangefinder2 >= 33) rangefinder2_good = true;
        
        moveLeft(255);
        
        // Delay for new range data
        readRangefinders(1, 20);
        readRangefinders(2, 50);
        
        if(rangefinder4 < 23) {
            moveBackward(255);
        } else if(rangefinder4 > 23) {
            moveForward(255);
        }
        
        readRangefinders(4, 50);
        moveLeft(255);
    }
    
    moveLeft(127);
    delay(100);
    
    while(!microswitch3) {
        moveLeft(127);
        delay(10);
        moveBackward(127);
        delay(40);
        readMicroswitches();
    }
    moveLeft(127);
    
    readMicroswitches();
    readRangefinders(5, 250);
    stopRobot();
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: toStageB1()                                                            //
// Description: Navigates to right stage B                                          //
//////////////////////////////////////////////////////////////////////////////////////

void toStageB1() {
    Serial2.println("D:TO_STAGE_B1");

    // Move right towards Stage B (1)
    moveRight(255);
    readMicroswitches();
    
    moveRight(255);
    delay(2000);
    
    boolean rangefinder0_good = false;
    boolean rangefinder3_good = false;
    
    // Move towards Stage B
    while(!rangefinder0_good || !rangefinder3_good) {
        if(rangefinder0 >= 33) rangefinder0_good = true;
        if(rangefinder3 >= 33) rangefinder3_good = true;
        
        moveRight(255);
        
        // Delay for new range data
        readRangefinders(0, 20);
        readRangefinders(3, 50);
        
        if(rangefinder4 < 23) {
            moveBackward(255);
        } else if(rangefinder4 > 23) {
            moveForward(255);
        }
        
        readRangefinders(4, 50);
        moveRight(255);
    }
    
    moveRight(127);
    delay(100);
    stopRobot();
    
    while(!microswitch2) {
        moveBackward(127);
        readMicroswitches();
    }
    
    moveRight(127);
    
    readMicroswitches();
    readRangefinders(5, 250);
    stopRobot();
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: StageB0()                                                              //
// Description: Activate left Stage B                                               //
//////////////////////////////////////////////////////////////////////////////////////

void stageB0() {
    Serial2.println("D:STAGE_B");
    
    servoStageB0.write(servoB0_pressed);
    //Serial2.println("D: Pressed: %d", servoB0_pressed);
    
    delay(1300);
    servoStageB0.write(servoB0_raised);
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: StageB1()                                                              //
// Description: Activate right Stage B                                              //
//////////////////////////////////////////////////////////////////////////////////////

void stageB1() {
    Serial2.println("D:STAGE_B");
    
    servoStageB1.write(servoB1_pressed);
    //Serial2.println("D: Pressed: %d", servoB1_pressed);
    
    delay(1300);
    servoStageB1.write(servoB1_raised);
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: toBooty0()                                                             //
// Description: Navigate to chest from left                                         //
//////////////////////////////////////////////////////////////////////////////////////

void toBooty0() {
    Serial2.println("D:TO_BOOTY_0");
    
    // Move towards front wall
    moveForward(255);
    while(rangefinder4 >= 15) {
        readRangefinders(4, 50);
    }
    
    // Straighten up against wall
    moveLeft(127);
    delay(500);
    
    // Move right towards chest
    moveRight(255);
    delay(1500);
    
    readRangefinders(5, 250);
    
    // FIXME: Verify distances on new sensors
    while((rangefinder0 <= 14) && (rangefinder3 <= 14)) {
        if(rangefinder4 < 14) {
            moveBackward(127);
            delay(50);
        } else if(rangefinder4 > 14) {
            moveForward(127);
            delay(50);
        }
        
        moveRight(127);
        
        if((rangefinder0 + rangefinder2) > (rangefinder1 + rangefinder3)) {
            turnLeft(127);
        } else if((rangefinder0 + rangefinder2) < (rangefinder1 + rangefinder3)) {
            turnRight(127);
        }
        
        delay(50);
        moveRight(127);
        readRangefinders(5, 250);
    }
    
    // Slow down when near chest
    int enough = 1;
    while((abs((rangefinder0 + rangefinder3) - (rangefinder1 + rangefinder2)) <= 1)) {
        if(abs((rangefinder0 + rangefinder3) - (rangefinder1 + rangefinder2)) <= 1) {
            enough--;
        }
        
        while(rangefinder4 < 14) {
            moveBackward(63);
            readRangefinders(4, 50);
        }
        
        while(rangefinder4 > 14) {
            moveForward(63);
            readRangefinders(4, 50);
        }
        
        moveLeft(63);
        readRangefinders(5, 250);
        
        // Fix if crooked
        if((rangefinder0 + rangefinder2) < (rangefinder1 + rangefinder3)) {
            turnRight(63);
        } else if((rangefinder0 + rangefinder2) > (rangefinder1 + rangefinder3)) {
            turnLeft(63);
        }
        
        // Move towards center
        if((rangefinder0 + rangefinder3) > (rangefinder1 +rangefinder2)) {
            moveLeft(65);
        } else if((rangefinder0 + rangefinder3) < (rangefinder1 +rangefinder2)) {
            moveRight(65);
        } else {
            stopRobot();
        }
        
        // Update rangefinders
        readRangefinders(5, 250);
    }
    
    stopRobot();
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: toBooty1()                                                             //
// Description: Navigate to chest from right                                        //
//////////////////////////////////////////////////////////////////////////////////////

void toBooty1() {
    Serial2.println("D:TO_BOOTY_1");
    
    // Move towards front wall
    moveForward(255);
    while(rangefinder4 > 15) {
        readRangefinders(4, 50);
    }
    
    // Straighten up against wall
    moveRight(127);
    delay(500);
    stopRobot();
    
    // Move right towards chest
    moveLeft(255);
    delay(1500);
    
    readRangefinders(5, 250);
    
    // FIXME: Verify distances on new sensors
    while((rangefinder1 <= 16) && (rangefinder2 <= 16)) {
        while(rangefinder4 < 15) {
            moveBackward(127);
            delay(50);
            moveLeft(127);
            
            readRangefinders(4, 50);
        }
        
        while(rangefinder4 > 15) {
            moveForward(127);
            delay(50);
            moveLeft(127);
            
            readRangefinders(4, 50);
        }
        
        if((rangefinder0 + rangefinder2) > (rangefinder1 + rangefinder3)) {
            turnLeft(127);
        } else if((rangefinder0 + rangefinder2) < (rangefinder1 + rangefinder3)) {
            turnRight(127);
        }
        
        delay(50);
        moveLeft(127);
        readRangefinders(5, 250);
    }
    
    // Slow down when near chest
    int enough = 1;
    while((abs((rangefinder0 + rangefinder3) - (rangefinder1 + rangefinder2)) <= 1)/* || (enough > 0)*/) {
        if(abs((rangefinder0 + rangefinder3) - (rangefinder1 + rangefinder2)) <= 1) {
            enough--;
        }
        
        while(rangefinder4 < 15) {
            moveBackward(63);
            delay(50);
            stopRobot();
            
            readRangefinders(4, 50);
        }
        
        while(rangefinder4 > 15) {
            moveForward(63);
            delay(50);
            stopRobot();
            
            readRangefinders(4, 50);
        }

        readRangefinders(5, 250);
        
        // Fix if crooked
        if((rangefinder0 + rangefinder2) < (rangefinder1 + rangefinder3)) {
            turnRight(63);
        } else if((rangefinder0 + rangefinder2) > (rangefinder1 + rangefinder3)) {
            turnLeft(63);
        }
        
        // Move towards center
        if((rangefinder0 + rangefinder3) > (rangefinder1 +rangefinder2)) {
            moveLeft(65);
        } else if((rangefinder0 + rangefinder3) < (rangefinder1 +rangefinder2)) {
            moveRight(65);
        } else {
            stopRobot();
        }
        
        // Update rangefinders
        readRangefinders(5, 250);
    }
    
    stopRobot();
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: retrieveBooty()                                                        //
// Description: Retrieve chest                                                      //
//////////////////////////////////////////////////////////////////////////////////////

void retrieveBooty() {
    Serial2.println("D:RETRIEVE_BOOTY");
      
    motorBooty->setSpeed(255);
    motorBooty->run(FORWARD);
    delay(1400);
    
    motorBooty->setSpeed(255);
    motorBooty->run(BACKWARD);
    delay(1800);
    
    motorBooty->run(RELEASE);
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: toFlag()                                                               //
// Description: Navigate to flag                                                    //
//////////////////////////////////////////////////////////////////////////////////////

void toFlag() {
    Serial2.println("D:TO_FLAG");
    
    moveForward(255);
    
    // Move to front wall
    while(!microswitch0 || !microswitch1) {
        if(microswitch0) {
            turnLeft(127);
        } else if(microswitch1) {
            turnRight(127);
        } else {
            moveForward(255);
        }
        readMicroswitches();
    }
    
    while(rangefinder2 != rangefinder3) {
        while(!microswitch0 || !microswitch1) {
            moveForward(127);
            readMicroswitches();
        }
        if(rangefinder2 > rangefinder3) {
            moveRight(127);
        } else if(rangefinder2 < rangefinder3) {
            moveLeft(127);
        }
        readRangefinders(2, 20);
        readRangefinders(3, 50);
        readMicroswitches();
    }
    
    while(!microswitch0 || !microswitch1) {
        if(microswitch0) {
            turnLeft(127);
        } else if(microswitch1) {
            turnRight(127);
        } else {
            moveForward(255);
        }
        readMicroswitches();
    }
    
    stopRobot();
    readRangefinders(5, 250);
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: raiseFlag()                                                            //
// Description: Raise the flag                                                      //
//////////////////////////////////////////////////////////////////////////////////////

void raiseFlag() {
    Serial2.println("D:RAISE_FLAG");
    
    motorStepper->step(100, BACKWARD, SINGLE);
    
    servoFlag.writeMicroseconds(2000);
    delay(6200);
    servoFlag.writeMicroseconds(1500);
    
    motorStepper->step(100, FORWARD, SINGLE);
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: toShip()                                                               //
// Description: Navigate back to the ship and against the back wall                 //
//////////////////////////////////////////////////////////////////////////////////////

void toShip() {
    Serial2.println("D:TO_SHIP");
     
    moveBackward(255);
    
    int enough = 1;
    while(rangefinder4 < 47 || (enough > 2)) {
        if(rangefinder4 > 47) {
            enough--;
        }

        readRangefinders(4, 50);
    }
    
    stopRobot();
    readRangefinders(5, 300);
    
    // Sums for centering in course
    int sum03 = rangefinder0 + rangefinder3;
    int sum12 = rangefinder1 + rangefinder2;
    
    // Sums for fixing angle
    int sum02 = rangefinder0 + rangefinder2;
    int sum13 = rangefinder1 + rangefinder3;
    
    // Make it happen captain
    while(abs(sum03 - sum12) > 1) {
        while(sum03 - sum12 > 1) {
            moveLeft(255);
            delay(250);
            stopRobot();
            readRangefinders(5, 250);
            
            // Sums for centering in course
            sum03 = rangefinder0 + rangefinder3;
            sum12 = rangefinder1 + rangefinder2;
        }
        
        while(sum12 - sum03 > 1) {
            moveRight(255);
            delay(250);
            stopRobot();
            readRangefinders(5, 250);
            
            // Sums for centering in course
            sum03 = rangefinder0 + rangefinder3;
            sum12 = rangefinder1 + rangefinder2;
        }
        
        while(abs(sum02 - sum13) > 1) {
            while(sum02 - sum13 > 1) {
                turnLeft(55);
                delay(250);
                stopRobot();
                readRangefinders(5, 250);
                        
                // Sums for fixing angle
                sum02 = rangefinder0 + rangefinder2;
                sum13 = rangefinder1 + rangefinder3;
            }
            
            while(sum13 - sum02 > 1) {
                turnRight(55);
                delay(250);
                stopRobot();
                readRangefinders(5, 250);
                        
                // Sums for fixing angle
                sum02 = rangefinder0 + rangefinder2;
                sum13 = rangefinder1 + rangefinder3;
            }
            
            readRangefinders(5, 300);
            
            // Sums for centering in course
            sum03 = rangefinder0 + rangefinder3;
            sum12 = rangefinder1 + rangefinder2;
            
            // Sums for fixing angle
            sum02 = rangefinder0 + rangefinder2;
            sum13 = rangefinder1 + rangefinder3;
        }
    }
    
    moveBackward(255);
    
    while(!microswitch2 || !microswitch3) {
        if(microswitch2) {
            turnLeft(127);
        } else if(microswitch3) {
            turnRight(127);
        } else {
            moveBackward(255);
        }
        
        readMicroswitches();
    }
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: toStageC0()                                                            //
// Description: Navigates to left stage C                                           //
//////////////////////////////////////////////////////////////////////////////////////

void toStageC0() {
    Serial2.println("D:TO_STAGE_C0");
    readRangefinders(1, 50);
    readMicroswitches();
    
    moveBackward(255);
    
    // Move to back wall
    while(!microswitch2 || !microswitch3) {
        if(microswitch2) {
            turnLeft(127);
        } else if(microswitch3) {
            turnRight(127);
        } else {
            moveBackward(255);
        }
        readMicroswitches();
    }
    
    Serial2.println("D:To Stage C0");
    
    // Move left towards Stage C. If leading microswitch is deactivated, robot
    // slides until contact is reestablished
    while(rangefinder1 < 33) {
        if(!microswitch3) {
            Serial2.println("D:slide");
            slideBackLeft(255);
        } else {
            Serial2.println("D:move");
            moveLeft(255);
        }

        readRangefinders(1, 50);
        readMicroswitches();
    }

    // Slow down when ~2 in away while maintaining contact with wall
    while(rangefinder1 < 35) {
        Serial2.println("D:close");
        while(!microswitch2 || !microswitch3) {
            moveBackward(127);
            readMicroswitches();
        }
        moveLeft(127);
        
        readRangefinders(1, 50);
        readMicroswitches();
    }

    // Straighten up on back wall
    moveBackward(255);
    Serial2.println("D:To back wall");
    while(!microswitch2 || !microswitch3) {
        if(microswitch2) {
            turnLeft(127);
        } else if(microswitch3) {
            turnRight(127);
        } else {
            moveBackward(255);
        }
        readMicroswitches();
    }
    
    stopRobot();
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: toStageC1()                                                            //
// Description: Navigates to right stage C                                          //
//////////////////////////////////////////////////////////////////////////////////////

void toStageC1() {
    Serial2.println("D:TO_STAGE_C1");
    readRangefinders(0, 50);
    readMicroswitches();
    
    moveBackward(255);
    
    // Move to back wall
    while(!microswitch2 || !microswitch3) {
        if(microswitch2) {
            turnLeft(127);
        } else if(microswitch3) {
            turnRight(127);
        } else {
            moveBackward(255);
        }
        readMicroswitches();
    }
    
    Serial2.println("D:To Stage C1");
    
    // Move right towards Stage C. If leading microswitch is deactivated, robot
    // slides until contact is reestablished
    while(rangefinder0 < 33) {
        if(!microswitch2) {
            slideBackRight(255);
        } else {
            moveRight(255); 
        }
        
        readRangefinders(0, 50);
        readMicroswitches();
    }

    // Slow down when ~2 in away while maintaining contact with wall
    while(rangefinder0 < 35) {
        Serial2.println("D:close");
        while(!microswitch2 || !microswitch3) {
            moveBackward(127);
            readMicroswitches();
        }
        moveRight(127);
        
        readRangefinders(0, 50);
        readMicroswitches();
    }

    // Straighten up on back wall
    moveBackward(255);
    Serial2.println("D:To back wall");
    while(!microswitch2 || !microswitch3) {
        if(microswitch2) {
            turnLeft(127);
        } else if(microswitch3) {
            turnRight(127);
        } else {
            moveBackward(255);
        }
        readMicroswitches();
    }
    moveLeft(255);
    
    // Press the shit out of that button
    delay(200);
    
    stopRobot();
}


//////////////////////////////////////////////////////////////////////////////////////
// Function: StageC()                                                               //
// Description: Activate Stage C                                                    //
//////////////////////////////////////////////////////////////////////////////////////

void stageC() {
    Serial2.println("D:STAGE_C");
}


///////////////////////
// Read Rangefinders //
///////////////////////

void readRangefinders(int number, int wait) {    
    if(number == 0) { 
        digitalWrite(RANGEFINDER_0_RX, HIGH);
        delay(35);
        digitalWrite(RANGEFINDER_0_RX, LOW);
        delay(wait);
        
        rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
        Serial2.print("R0:");
        Serial2.println(rangefinder0);
    }
    
    if(number == 1) { 
        digitalWrite(RANGEFINDER_1_RX, HIGH);
        delay(35);
        digitalWrite(RANGEFINDER_1_RX, LOW);
        delay(wait);
        
        rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
        Serial2.print("R1:");
        Serial2.println(rangefinder1);
    }
    
    if(number == 2) { 
        digitalWrite(RANGEFINDER_2_RX, HIGH);
        delay(35);
        digitalWrite(RANGEFINDER_2_RX, LOW);
        delay(wait);
        
        rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
        Serial2.print("R2:");
        Serial2.println(rangefinder2);
    }
    
    if(number == 3) { 
        digitalWrite(RANGEFINDER_3_RX, HIGH);
        delay(35);
        digitalWrite(RANGEFINDER_3_RX, LOW);
        delay(wait);
        
        rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
        Serial2.print("R3:");
        Serial2.println(rangefinder3);
    }
    
    if(number == 4) { 
        digitalWrite(RANGEFINDER_4_RX, HIGH);
        delay(35);
        digitalWrite(RANGEFINDER_4_RX, LOW);
        delay(wait);
        
        rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
        Serial2.print("R4:");
        Serial2.println(rangefinder4);
    }
    
    if(number == 5) {
        digitalWrite(RANGEFINDER_0_RX, HIGH);        
        delay(30);        
        digitalWrite(RANGEFINDER_0_RX, LOW);        
        delay(wait/5);
        
        rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
        Serial2.print("R0:");
        Serial2.println(rangefinder0);
        
        digitalWrite(RANGEFINDER_1_RX, HIGH);        
        delay(30);        
        digitalWrite(RANGEFINDER_1_RX, LOW);        
        delay(wait/5);
        
        rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
        Serial2.print("R1:");
        Serial2.println(rangefinder1);
        
        digitalWrite(RANGEFINDER_2_RX, HIGH);        
        delay(30);       
        digitalWrite(RANGEFINDER_2_RX, LOW);        
        delay(wait/5);
    
        rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
        Serial2.print("R2:");
        Serial2.println(rangefinder2);
        
        digitalWrite(RANGEFINDER_3_RX, HIGH);        
        delay(30);        
        digitalWrite(RANGEFINDER_3_RX, LOW);        
        delay(wait/5);
    
        rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
        Serial2.print("R3:");
        Serial2.println(rangefinder3);
        
        digitalWrite(RANGEFINDER_4_RX, HIGH);        
        delay(30);        
        digitalWrite(RANGEFINDER_4_RX, LOW);        
        delay(wait/5);
    
        rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
        Serial2.print("R4:");
        Serial2.println(rangefinder4);
    }
}


////////////////////////
// Read Microswitches //
////////////////////////

void readMicroswitches() {
    microswitch0 = digitalRead(MICROSWITCH_0);
    Serial2.print("U0:");
    Serial2.println(microswitch0);
    
    microswitch1 = digitalRead(MICROSWITCH_1);
    Serial2.print("U1:");
    Serial2.println(microswitch1);
    
    microswitch2 = digitalRead(MICROSWITCH_2);
    Serial2.print("U2:");
    Serial2.println(microswitch2);
    
    microswitch3 = digitalRead(MICROSWITCH_3);
    Serial2.print("U3:");
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


//////////////////////
// Pivot Robot Left //
//////////////////////

void pivotLeft(int velocity) {
    motor0_commandVelocity = velocity;
    motor1_commandVelocity = 0;
    motor2_commandVelocity = 0;
    motor3_commandVelocity = velocity;
    motor0->run(BACKWARD);
    motor3->run(BACKWARD);
    commandMotors();
}


///////////////////////
// Pivot Robot Right //
///////////////////////

void pivotRight(int velocity) {
    motor0_commandVelocity = 0;
    motor1_commandVelocity = velocity;
    motor2_commandVelocity = velocity;
    motor3_commandVelocity = 0;
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);

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
    motor0->setSpeed(0);
    motor1->setSpeed(0);
    motor2->setSpeed(0);
    motor3->setSpeed(0);
    motor0->run(RELEASE);
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);
}

