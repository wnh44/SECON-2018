
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


///////////////////////////
// Rangefinder Variables //
///////////////////////////

int rangefinder0;
int rangefinder1;
int rangefinder2;
int rangefinder3;
int rangefinder4;


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
    
    // Initialize Start Button Pin
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

    digitalWrite(RANGEFINDER_0_RX, HIGH);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, LOW);
}

void loop() {
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
    Serial.println("\n\nWAIT_FOR_START");
    
    while(digitalRead(START_BUTTON)) {
        delay(100);
    }
    
    state = DECODE_LED;
}


////////////////
// Decode LED //
////////////////

void decodeLED() {
    Serial.println("\nDECODE_LED");
    
    // FIXME: Returns a random number for now
    randomSeed(analogRead(A14));
    for(int i = 0; i < 3; i++) {
        locations[i] = random(0, 2);
    }
    locations[1] = 0;
    
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
    Serial.println("\nTO_STAGE_A");
    
    moveBackward(255);
    
    // Move to back wall
    while(!digitalRead(MICROSWITCH_2) || !digitalRead(MICROSWITCH_3)) {
        if(digitalRead(MICROSWITCH_2)) {
            turnLeft(127);
        } else if(digitalRead(MICROSWITCH_3)) {
            turnRight(127);
        } else {
            moveBackward(255);
        }
    }
    
    if(locations[0] == 0) {
        // Move left towards Stage A    
        while((analogRead(RANGEFINDER_1) - 3) / 2 + 3 < 36) {
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
    
        // Slow down when ~2 in away
        while((analogRead(RANGEFINDER_1) - 3) / 2 + 3 < 38) {
            while(!digitalRead(MICROSWITCH_2) || !digitalRead(MICROSWITCH_3)) {
                moveBackward(127);
            }
            
            moveLeft(127);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(300);
        }
    } else {
        // Move right towards Stage A    
        while((analogRead(RANGEFINDER_0) - 3) / 2 + 3 < 35) {
            if(!digitalRead(MICROSWITCH_2)) {
                slideBackRight(255);
            } else {
                moveRight(255);
            }
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            
            delay(300);
        }
    
        // Slow down when ~2 in away
        while((analogRead(RANGEFINDER_0) - 3) / 2 + 3 < 36) {
            while(!digitalRead(MICROSWITCH_2) || !digitalRead(MICROSWITCH_3)) {
                moveBackward(127);
            }
            
            moveRight(127);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(300);
        }
    }
       
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
    
    stopRobot();
    
    state = STAGE_A;
}

void stageA() {
    Serial.println("\nSTAGE_A");
    
    // FIXME: Stage A implementation
    state = FROM_STAGE_A;
}

void fromStageA() {
    Serial.println("\nFROM_STAGE_A");
    
    if(locations[0] == 0) {
        // Move right towards center    
        while(((analogRead(RANGEFINDER_1) - 3) / 2 + 3) - ((analogRead(RANGEFINDER_0) - 3) / 2 + 3) >= 4) {
            if(!digitalRead(MICROSWITCH_2)) {
                slideBackRight(255);
            } else {
                moveRight(255);
            }
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            
            delay(300);
        }
    
        // Slow down when ~2 in away
        while(((analogRead(RANGEFINDER_1) - 3) / 2 + 3) != ((analogRead(RANGEFINDER_0) - 3) / 2 + 3)) {
            while(!digitalRead(MICROSWITCH_2) || !digitalRead(MICROSWITCH_3)) {
                moveBackward(127);
            }
            moveRight(65);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(300);
        }
    } else {
        // Move left towards center
        int twice = 0;
        while(((analogRead(RANGEFINDER_0) - 3) / 2 + 3) - ((analogRead(RANGEFINDER_1) - 3) / 2 + 3) >= 2 || twice == 0) {
            if(((analogRead(RANGEFINDER_0) - 3) / 2 + 3) - ((analogRead(RANGEFINDER_1) - 3) / 2 + 3) <= 2) {
                twice = 1;
            }
            Serial.print((analogRead(RANGEFINDER_0) - 3) / 2 + 3);
            Serial.print(" ");
            Serial.print((analogRead(RANGEFINDER_1) - 3) / 2 + 3);
            Serial.print(" ");
            Serial.println(twice);
            
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
        Serial.print((analogRead(RANGEFINDER_0) - 3) / 2 + 3);
        Serial.print(" ");
        Serial.println((analogRead(RANGEFINDER_1) - 3) / 2 + 3);
            
    
        // Slow down when ~2 in away
        while(((analogRead(RANGEFINDER_0) - 3) / 2 + 3) != ((analogRead(RANGEFINDER_1) - 3) / 2 + 3)) {
            while(!digitalRead(MICROSWITCH_2) || !digitalRead(MICROSWITCH_3)) {
                moveBackward(127);
            }
            moveLeft(65);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(300);
        }
    }
    // For now
    stopRobot();
    
    state = TO_STAGE_B;
}

void toStageB() {
    Serial.println("\nTO_STAGE_B");
    
    moveForward(255);
    delay(12000); // Probably necessary
    
    // Navigate towards chest
    while((analogRead(RANGEFINDER_4) - 3) / 2 + 3 >= 17) {
        rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
        rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
        rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
        rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
        
        int leftSum = rangefinder0 + rangefinder3;
        int rightSum = rangefinder1 + rangefinder2;
        
        while((leftSum - rightSum) >= 4) {
            moveLeft(255);
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(100);
            moveForward(255);
            delay(200);
            
            rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
            rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
            rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
            rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
            
            leftSum = rangefinder0 + rangefinder3;
            rightSum = rangefinder1 + rangefinder2;
        }
        
        while((rightSum - leftSum) >= 4) {
            moveRight(255);
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(100);
            moveForward(255);
            delay(200);
            
            rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
            rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
            rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
            rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
            
            leftSum = rangefinder0 + rangefinder3;
            rightSum = rangefinder1 + rangefinder2;
        }
            
        
        if((rangefinder0 + rangefinder2) > (rangefinder3 + rangefinder1)) {
            turnLeft(255);
        } else if((rangefinder0 + rangefinder2) < (rangefinder3 + rangefinder1)) {
            turnRight(255);
        }
            
        digitalWrite(RANGEFINDER_0_RX, 1);
        delay(35);
        digitalWrite(RANGEFINDER_0_RX, 0);
        delay(100);
        moveForward(255);
        delay(200);
    }
    stopRobot();
    
    digitalWrite(RANGEFINDER_0_RX, 1);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, 0);
    delay(300);
    
    rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
    rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
    rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
    rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
    
    // Straighten up
    int enough = 0;
    while((abs((rangefinder0 + rangefinder2) - (rangefinder3 + rangefinder1)) <= 1) || (enough < 1)) {
        if(abs((rangefinder0 + rangefinder2) - (rangefinder3 + rangefinder1)) <= 1) {
            enough++;
        }  
      
        digitalWrite(RANGEFINDER_0_RX, 1);
        delay(35);
        digitalWrite(RANGEFINDER_0_RX, 0);
        
        if((rangefinder0 + rangefinder2) > (rangefinder3 + rangefinder1)) {
            turnLeft(63);
            delay(150);
            stopRobot();
        } else if((rangefinder0 + rangefinder2) < (rangefinder3 + rangefinder1)) {
            turnRight(63);
            delay(150);
            stopRobot();
        }
        
        delay(300);

        rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
        rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
        rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
        rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
    }
    
    moveLeft(255);

    rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
    rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
    
    // Move towards Stage B
    while((rangefinder1 < 30) || (rangefinder2 < 30)) {
        Serial.print(rangefinder1);
        Serial.print(" ");
        Serial.println(rangefinder2);
        
        if((rangefinder1 - rangefinder2) >= 2) {
            turnLeft(255);
            delay(100);
        } else if((rangefinder2 - rangefinder1) >= 2) {
            turnRight(100);
            delay(50);
        }
        moveLeft(255);
        
        digitalWrite(RANGEFINDER_0_RX, 1);
        delay(35);
        digitalWrite(RANGEFINDER_0_RX, 0);
        delay(300);
        
        rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
        rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
    }
        Serial.print(rangefinder1);
        Serial.print(" ");
        Serial.println(rangefinder2);
    
    moveLeft(127);
    
    // Slow down when ~3 in away
    while((rangefinder1 < 33) && (rangefinder2 < 33)) {
        if(rangefinder1 < rangefinder2) {
            turnLeft(127);
            delay(50);
        } else if(rangefinder1 > rangefinder2) {
            turnRight(127);
            delay(50);
        }
        moveLeft(127);
        
        digitalWrite(RANGEFINDER_0_RX, 1);
        delay(35);
        digitalWrite(RANGEFINDER_0_RX, 0);
        delay(300);
        
        rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
        rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
    }
    turnRight(255);
    
    digitalWrite(RANGEFINDER_0_RX, 1);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, 0);
    delay(300);
    stopRobot();
    
    state = STAGE_B;
}

void stageB() {
    Serial.println("\nSTAGE_B");
    
    state = TO_BOOTY;
}

void toBooty() {
    Serial.println("\nTO_BOOTY");
    
    // Move towards back wall
    moveForward(255);
    while(((analogRead(RANGEFINDER_4) - 3) / 2 + 3) > 16) {
        if(rangefinder1 > 33 && rangefinder2 > 33) {
            moveRight(255);
        } else if(rangefinder1 > 33) {
            turnRight(255);
        } else if((rangefinder2 > 33 && rangefinder1) > 20) {
            turnLeft(255);
        } 
        
        digitalWrite(RANGEFINDER_0_RX, 1);
        delay(35);
        digitalWrite(RANGEFINDER_0_RX, 0);
        delay(100);
        moveForward(255);
        delay(200);
        
        rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
        rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
    }
    
    rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
    rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
    rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
    rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
    rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
    
    moveLeft(255); 
    delay(3000);
    
    // Move right towards chest
    moveRight(255);
    delay(2000);
    /*int time = millis();
    while(millis() - time < 2000) {
        Serial.println(millis() - time);
        while(rangefinder4 < 16) {
            moveBackward(255);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(100);
            moveRight(255);
            delay(200);
        
            rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
        }
        
        while(rangefinder4 > 16) {
            moveForward(255);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(100);
            moveRight(255);
            delay(200);
        
            rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
        }
    }*/
    
    digitalWrite(RANGEFINDER_0_RX, 1);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, 0);
    delay(100);
    
    rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
    rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
    rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
    rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
    
    while((rangefinder0 <= 16) && (rangefinder3 <= 16)) {
        while(rangefinder4 < 16) {
            moveBackward(255);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(100);
            moveRight(255);
            delay(200);
        
            rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
        }
        
        while(rangefinder4 > 16) {
            moveForward(255);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(100);
            moveRight(255);
            delay(200);
        
            rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
        }
        
        Serial.print(rangefinder0);
        Serial.print(" ");
        Serial.println(rangefinder3);
        
        if((rangefinder0 + rangefinder2) > (rangefinder1 + rangefinder3)) {
            turnLeft(255);
        } else if((rangefinder0 + rangefinder2) < (rangefinder1 + rangefinder3)) {
            turnRight(255);
        }
            
        digitalWrite(RANGEFINDER_0_RX, 1);
        delay(35);
        digitalWrite(RANGEFINDER_0_RX, 0);
        delay(100);
        moveRight(255);
        delay(150);
        
        rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
        rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
        rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
        rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
        rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
    }
    
    int enough = 0;
    // Slow down when near chest
    while((abs((rangefinder0 + rangefinder3) - (rangefinder1 + rangefinder2)) <= 1) || (enough < 1)) {
        if(abs((rangefinder0 + rangefinder3) - (rangefinder1 + rangefinder2)) <= 1) {
            enough++;
        }
        while(rangefinder4 < 16) {
            moveBackward(127);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(100);
            moveRight(127);
            delay(200);
        
            rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
        }
        
        while(rangefinder4 > 16) {
            moveForward(127);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(100);
            moveRight(127);
            delay(200);
        
            rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
        }
        
        
        if((rangefinder0 + rangefinder2) < (rangefinder1 + rangefinder3)) {
            turnRight(127);
        } else if((rangefinder0 + rangefinder2) > (rangefinder1 + rangefinder3)) {
            turnLeft(127);
        }
            
        digitalWrite(RANGEFINDER_0_RX, 1);
        delay(35);
        digitalWrite(RANGEFINDER_0_RX, 0);
        delay(150);
        moveRight(127);
        delay(150);
        
        rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
        rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
        rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
        rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
        rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
    }
    
    stopRobot();
    digitalWrite(RANGEFINDER_0_RX, 1);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, 0);
    delay(300);
    
    state = RETRIEVE_BOOTY;
}

void retrieveBooty() {
    Serial.println("RETRIEVE_BOOTY");
    
    state = TO_FLAG;
}

void toFlag() {
    Serial.println("\nTO_FLAG");
    
    moveForward(255);
    
    // Move to front wall
    while(!digitalRead(MICROSWITCH_0) || !digitalRead(MICROSWITCH_1)) {
        if(digitalRead(MICROSWITCH_0)) {
            turnLeft(127);
        } else if(digitalRead(MICROSWITCH_1)) {
            turnRight(127);
        } else {
            moveForward(255);
        }
    }
    
    stopRobot();
    
    state = RAISE_FLAG;
}

void raiseFlag() {
    Serial.println("\nRAISE_FLAG");
    
    while(digitalRead(START_BUTTON)) {
        delay(100);
    }
    
    state = TO_SHIP;
}

void toShip() {
    Serial.println("\nTO_SHIP");
    
    
    moveBackward(255);
    
    rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
    
    int enough = 0;
    while(rangefinder4 < 45 || (enough < 1)) {
        Serial.println(rangefinder4);
        if(rangefinder4 > 45) {
            enough++;
        }

        digitalWrite(RANGEFINDER_0_RX, 1);
        delay(35);
        digitalWrite(RANGEFINDER_0_RX, 0);
        delay(300);
        
        rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
    }
    
    stopRobot();
    
    digitalWrite(RANGEFINDER_0_RX, 1);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, 0);
    delay(300);
    
    turnRight(255);
    delay(1000);
    
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
    state = TO_STAGE_C;
}

void toStageC() {
    Serial.println("\nTO_STAGE_C");
    
    if(locations[2] == 0) {
        // Move left towards Stage C    
        while((analogRead(RANGEFINDER_1) - 3) / 2 + 3 < 36) {
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
    
        // Slow down when ~2 in away
        while((analogRead(RANGEFINDER_1) - 3) / 2 + 3 < 38) {
            while(!digitalRead(MICROSWITCH_2) || !digitalRead(MICROSWITCH_3)) {
                moveBackward(127);
            }
            
            moveLeft(127);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(300);
        }
    } else {
        // Move right towards Stage C    
        while((analogRead(RANGEFINDER_0) - 3) / 2 + 3 < 35) {
            if(!digitalRead(MICROSWITCH_2)) {
                slideBackRight(255);
            } else {
                moveRight(255);
            }
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            
            delay(300);
        }
    
        // Slow down when ~2 in away
        while((analogRead(RANGEFINDER_0) - 3) / 2 + 3 < 37) {
            while(!digitalRead(MICROSWITCH_2) || !digitalRead(MICROSWITCH_3)) {
                moveBackward(127);
            }
            
            moveRight(127);
            
            digitalWrite(RANGEFINDER_0_RX, 1);
            delay(35);
            digitalWrite(RANGEFINDER_0_RX, 0);
            delay(300);
        }
    }
    
    stopRobot();
    
    state = STAGE_C;
}

void stageC() {
    Serial.println("STAGE_C");
    
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

