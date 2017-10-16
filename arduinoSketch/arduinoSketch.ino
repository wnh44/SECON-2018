/*
 * Serial data is sent in the following manner:
 *
 * {operation}{mode}{number}{:}{quantity}{:}{value}
 * 
 * {operation}
 *   M: Set motor speed
 *   P: Set pin mode
 *   R: Read value
 *   W: Write value
 *   
 * {mode}
 *   A: Analog
 *   D: Digital
 *   I: INPUT
 *   O: OUTPUT
 *   P: INPUT_PULLUP
 *   
 * {number}
 *   If M: Motor number to set.
 *   If P: N/A
 *   If R: Pin number to read from.
 *   If W: Pin number to write to.
 *   
 * {quantity}
 *   If M: Number of (sequential) motors to set.
 *   If P: N/A
 *   If R: Number of (sequential) inputs to read.
 *   If W: N/A
 *   
 * {value}
 *   If M: Motor speed to set.
 *   If P: N/A
 *   If R: N/A
 *   If W: Value to write.
 * 
 * Examples:
 *   Set Pin Mode:  PI4
 *   Digital Read:  DR7:3
 *   Digital Write: DW4:0
 *   Analog Read:   AR4:0
 *   Analog Write:  AW0:759
 */

char operation, mode, type;
int pinNumber, digitalValue, analogValue, writeValue;
int pause = 5;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
  pinMode(4, OUTPUT);
}

void pinModeLocal(int pinNumber, char type, char mode) {
  char pinNum;
  if(type == 'A') {
    pinNum = 'A' + pinNumber;
  } else {
    pinNum = pinNumber;
  }
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

void digitalReadLocal(int pinNumber) {
  digitalValue = digitalRead(pinNumber);
  Serial.println(digitalValue);
}

void digitalWriteLocal(int pinNumber, int digitalValue) {
  digitalWrite(pinNumber, digitalValue);
}

void analogReadLocal(int pinNumber) {
  analogValue = analogRead(pinNumber);
  Serial.println(analogValue);
}

void analogWriteLocal(int pinNumber, int analogValue) {
  analogWrite(pinNumber, analogValue);
}

void loop() {
  // FIXME: Update with motor control and quantity
  if(Serial.available() > 0) {
    operation = Serial.read();
    delay(pause);
    mode = Serial.read();
    if(operation == 'M') {
      type = Serial.read();
    }
    pinNumber = Serial.parseInt();
    if(Serial.read() == ':') {
      writeValue = Serial.parseInt();
    }
    switch(operation) {
      case 'R':
        if (mode == 'D') {
          digitalReadLocal(pinNumber);
        } else if (mode == 'A') {
          digitalWrite(4, 1);
          analogReadLocal(pinNumber);
        } else {
          break;
        }
        break;
        
      case 'W':
        if (mode == 'D') {
          digitalWriteLocal(pinNumber, writeValue);
        } else if (mode == 'A') {
          analogWriteLocal(pinNumber, writeValue);
        } else {
          break;
        }
        break;

      case 'M':
        pinModeLocal(pinNumber,type, mode);
        break;

      default:
        break;
    }
  }
}
