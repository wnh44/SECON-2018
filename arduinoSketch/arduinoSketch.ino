/*
 *  Serial data is sent in the following manner:
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
 *    I: INPUT
 *    O: OUTPUT
 *    P: INPUT_PULLUP
 *    
 *  {number}
 *    If M: Motor number to set.
 *    If P: N/A
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
 */

#include <Wire.h>
#include "utility/Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"

char operation, mode;
int index, quantity, value;

int digitalValue, analogValue;
int pause = 5;

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(100);
    pinMode(4, OUTPUT);
}

void pinModeLocal(int pinNumber, char mode) {
    switch(mode){
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

void digitalReadLocal(int pinNumber, int quantity) {
    for (int i = 0; i < quantity - 1; i++) {
        digitalValue = digitalRead(pinNumber + i);
        Serial.print(digitalValue);
        Serial.print(':');
    }
    digitalValue = digitalRead(pinNumber + quantity - 1);
    Serial.println(digitalValue);
}

void digitalWriteLocal(int pinNumber, int digitalValue) {
    digitalWrite(pinNumber, digitalValue);
}

void analogReadLocal(int pinNumber, int quantity) {
    for (int i = 0; i < quantity - 1; i++) {
        analogValue = analogRead(pinNumber);
        Serial.print(analogValue);
        Serial.print(':');
    }
    analogValue = analogRead(pinNumber + quantity - 1);
    Serial.println(analogValue);
}

void analogWriteLocal(int pinNumber, int analogValue) {
    analogWrite(pinNumber, analogValue);
}

void setMotorSpeedLocal(char mode, int motorNumber, int quantity, int value) {
    // FIXME
}

void loop() {
    if (Serial.available() > 0) {
        operation = Serial.read();
        delay(pause); // May be necessary elsewhere
        mode = Serial.read();
        index = Serial.parseInt();
        if (Serial.read() == ':') {
            if (operation == 'W') {
                value = Serial.parseInt();
            } else {
                quantity = Serial.parseInt();
            }
            if (operation == 'M') {
                value = Serial.parseInt();
            }
        }
        
        switch(operation) {
            case 'M':
                setMotorSpeedLocal(mode, index, quantity, value);
                break;
      
            case 'P':
                pinModeLocal(index, mode);
                break;
                
            case 'R':
                if (mode == 'D') {
                    digitalReadLocal(index, quantity);
                } else if (mode == 'A') {
                    analogReadLocal(index, quantity);
                } else {
                    break;
                }
                break;
              
            case 'W':
                if (mode == 'D') {
                    digitalWriteLocal(index, value);
                } else if (mode == 'A') {
                    analogWriteLocal(index, value);
                } else {
                    break;
                }
                break;
      
            default:
                break;
        }
    }
}
