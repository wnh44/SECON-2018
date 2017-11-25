
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
#define RANGEFINDER_0_RX 0

// Start Button
#define START_BUTTON 2

void setup() {
    Serial.begin(57600);
    Serial.setTimeout(500);
    
    // Initialize Microswitch Pins
    pinMode(MICROSWITCH_0, INPUT_PULLUP);
    pinMode(MICROSWITCH_1, INPUT_PULLUP);
    pinMode(MICROSWITCH_2, INPUT_PULLUP);
    pinMode(MICROSWITCH_3, INPUT_PULLUP);
    pinMode(START_BUTTON, INPUT_PULLUP);

    delay(250);
    digitalWrite(RANGEFINDER_0_RX, HIGH);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, LOW);
}

void loop() {    
    Serial.print(analogRead(RANGEFINDER_0));
    Serial.print(" ");
    Serial.print(analogRead(RANGEFINDER_1));
    Serial.print(" ");
    Serial.print(analogRead(RANGEFINDER_2));
    Serial.print(" ");
    Serial.print(analogRead(RANGEFINDER_3));
    Serial.print(" ");
    Serial.println(analogRead(RANGEFINDER_4));
}
