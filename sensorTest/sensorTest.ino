
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

char formattedOutput[4];

void setup() {
    Serial.begin(9600);
    
    // Initialize Microswitch Pins
    pinMode(MICROSWITCH_0, INPUT_PULLUP);
    pinMode(MICROSWITCH_1, INPUT_PULLUP);
    pinMode(MICROSWITCH_2, INPUT_PULLUP);
    pinMode(MICROSWITCH_3, INPUT_PULLUP);
    pinMode(START_BUTTON, INPUT_PULLUP);

    pinMode(RANGEFINDER_0_RX, OUTPUT);
    
    digitalWrite(RANGEFINDER_0_RX, 1);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, 0);
    
    delay(300);
}

void loop() {
    Serial.print("RF0: ");
    sprintf(formattedOutput, "%03d", (analogRead(RANGEFINDER_0) - 3) / 2 + 3);
    Serial.print(formattedOutput);
    
    Serial.print("  RF1: ");
    sprintf(formattedOutput, "%03d", (analogRead(RANGEFINDER_1) - 3) / 2 + 3);
    Serial.print(formattedOutput);
    
    Serial.print("  RF2: ");
    sprintf(formattedOutput, "%03d", (analogRead(RANGEFINDER_2) - 3) / 2 + 3);
    Serial.print(formattedOutput);
    
    Serial.print("  RF3: ");
    sprintf(formattedOutput, "%03d", (analogRead(RANGEFINDER_3) - 3) / 2 + 3);
    Serial.print(formattedOutput);
    
    Serial.print("  RF4: ");
    sprintf(formattedOutput, "%03d", (analogRead(RANGEFINDER_4) - 3) / 2 + 3);
    Serial.print(formattedOutput);
    
    Serial.print("   US0: ");
    Serial.print(digitalRead(MICROSWITCH_0));
    
    Serial.print("  US1: ");
    Serial.print(digitalRead(MICROSWITCH_1));
    
    Serial.print("  US2: ");
    Serial.print(digitalRead(MICROSWITCH_2));
    
    Serial.print("  US3: ");
    Serial.print(digitalRead(MICROSWITCH_3));
    
    Serial.print("   SB: ");
    Serial.println(digitalRead(START_BUTTON));
    
    
    digitalWrite(RANGEFINDER_0_RX, 1);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, 0);
    delay(500);
}
