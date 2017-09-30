char operation;
char mode;
char type;
int pinNumber;
int digitalValue;
int analogValue;
int writeValue;
int pause = 5;

/* 
 *  Serial data is received in the following manner:
 *     operation mode pinNumber : writeValue
 * 
 *  Examples:
 *     Set Pin Mode:  MI4
 *     Digital Read:  DR7
 *     Digital Write: DW4:0
 *     Analog Read:   AR4
 *     Analog Write:  AW0:759
 * 
 */

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
  digitalWrite(4, 0);
  if(Serial.available() > 0) {
    operation = Serial.read();
    delay(pause);
    mode = Serial.read();
    if(mode == 'M') {
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
