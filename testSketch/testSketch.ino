
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

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(500);
    
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
    
    delay(250);
}

void loop() {
    Serial.print((analogRead(RANGEFINDER_0) - 3) / 2 + 3);
    Serial.print(" ");
    Serial.print((analogRead(RANGEFINDER_1) - 3) / 2 + 3);
    Serial.print(" ");
    Serial.print((analogRead(RANGEFINDER_2) - 3) / 2 + 3);
    Serial.print(" ");
    Serial.print((analogRead(RANGEFINDER_3) - 3) / 2 + 3);
    Serial.print(" ");
    Serial.print((analogRead(RANGEFINDER_4) - 3) / 2);

    Serial.print("   ");
    Serial.print(digitalRead(MICROSWITCH_0));
    Serial.print(digitalRead(MICROSWITCH_1));
    Serial.print(digitalRead(MICROSWITCH_2));
    Serial.println(digitalRead(MICROSWITCH_3));

    digitalWrite(RANGEFINDER_0_RX, 1);
    delay(35);
    digitalWrite(RANGEFINDER_0_RX, 0);
    delay(500);
}
