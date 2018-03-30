
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

// Rangefinder RX
#define RANGEFINDER_0_RX 23
#define RANGEFINDER_1_RX 25
#define RANGEFINDER_2_RX 27
#define RANGEFINDER_3_RX 29
#define RANGEFINDER_4_RX 31

// Start Button
#define START_BUTTON 2


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

int startButton = 0;


char formattedOutput[4];

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

    pinMode(RANGEFINDER_0_RX, OUTPUT);
    
    digitalWrite(RANGEFINDER_0_RX, 1);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, 0);
    
    delay(300);
}

void loop() {
    //////////////////
    // Rangefinders //
    //////////////////
    
    digitalWrite(RANGEFINDER_0_RX, HIGH);        
    delay(30);
    digitalWrite(RANGEFINDER_0_RX, LOW);
    delay(50);
    
    rangefinder0 = (analogRead(RANGEFINDER_0) - 3) / 2 + 3;
    Serial2.print("R0:");
    Serial2.println(rangefinder0);
    
    digitalWrite(RANGEFINDER_1_RX, HIGH);        
    delay(30);
    digitalWrite(RANGEFINDER_1_RX, LOW);
    delay(50);
    
    rangefinder1 = (analogRead(RANGEFINDER_1) - 3) / 2 + 3;
    Serial2.print("R1:");
    Serial2.println(rangefinder1);
    
    digitalWrite(RANGEFINDER_2_RX, HIGH);        
    delay(30);
    digitalWrite(RANGEFINDER_2_RX, LOW);
    delay(50);

    rangefinder2 = (analogRead(RANGEFINDER_2) - 3) / 2 + 3;
    Serial2.print("R2:");
    Serial2.println(rangefinder2);
    
    digitalWrite(RANGEFINDER_3_RX, HIGH);        
    delay(30);
    digitalWrite(RANGEFINDER_3_RX, LOW);
    delay(50);

    rangefinder3 = (analogRead(RANGEFINDER_3) - 3) / 2 + 3;
    Serial2.print("R3:");
    Serial2.println(rangefinder3);
    
    digitalWrite(RANGEFINDER_4_RX, HIGH);        
    delay(30);
    digitalWrite(RANGEFINDER_4_RX, LOW);
    delay(50);

    rangefinder4 = (analogRead(RANGEFINDER_4) - 3) / 2 + 3;
    Serial2.print("R4:");
    Serial2.println(rangefinder4);
    
    
    ///////////////////
    // Microswitches //
    ///////////////////
    
    // Read microswitch values
    microswitch0 = digitalRead(MICROSWITCH_0);
    microswitch1 = digitalRead(MICROSWITCH_1);
    microswitch2 = digitalRead(MICROSWITCH_2);
    microswitch3 = digitalRead(MICROSWITCH_3);
    startButton = digitalRead(START_BUTTON);
    
    // Microswitch 0
    Serial.print("   US0: ");
    Serial.print(microswitch0);
    Serial2.print("U0:");
    Serial2.println(microswitch0);
    
    // Microswitch 1
    Serial.print("  US1: ");
    Serial.print(microswitch1);
    Serial2.print("U1:");
    Serial2.println(microswitch1);
    
    // Microswitch 2
    Serial.print("  US2: ");
    Serial.print(microswitch2);
    Serial2.print("U2:");
    Serial2.println(microswitch2);
    
    // Microswitch 3
    Serial.print("  US3: ");
    Serial.print(microswitch3);
    Serial2.print("U3:");
    Serial2.println(microswitch3);
    
    // Start button
    Serial.print("   SB: ");
    Serial.println(startButton);
    
    Serial2.print("D:Debug test ");
    Serial2.println(millis());
}
