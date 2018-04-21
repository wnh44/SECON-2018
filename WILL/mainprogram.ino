/***********************************************************************************************
 * 
 * Another serial would have to be added to read to the computer instead of an 
 * OLED screen
 * 
 * Ultrasonic Vcc connected to 5V, Trig to Arduino pin 9, Echo to Arduino
 * pin 10, Gnd to arduino ground
 * 
 * liDAR: Black (leftmost when facing) connected to arduino ground, Red
 * connected to 5V, white connected to TXO->1 for communication from Arduino
 * to sensor (both connected to same pin), green connected to RX1 or RX2 
 * (one for each sensor)
 * 
 * OLED: SDA connected to Arduino SDA 20, SCL connected to Arduino SCL 21,
 * RST connected to Arduino pin 9, GND connected to arduino ground, 3.3V
 * connected to Arduino 3.3V, VIN connected to Arduino 5V
 * 
 * Power usage seems to necessitate connection to power through a USB cord
 * instead of an actual power cord
 * 
 * Testing ongoing, but liDARs freak out sometimes with different colors 
 * (specifically black) and Ultrasonic not quite as accurate
 *
 ***********************************************************************************************
 *
 * Just point the IR blaster at the sensor and it should pick it up
 * and display on the OLED
 * 
 * IR blaster should be connected to pin 9 on arduino with about 470 OHM
 * resister connecting it to ground
 * 
 * IR receiver pins are [Output, Ground, 5V] when it is facing you
 * 
 * OLED: SDA connected to SDA 20 on Mega, SCL connected to SCL 21,
 * RST connected to pin 4, GND connected to ground, 3.3V connected to
 * 3.3V, and VIN connected to 5V
 *
 ***********************************************************************************************/


//OLED Screen Setup//////////////////////////////////////
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 22            //Connect OLED RST pin to pin 4 on arduino
Adafruit_SSD1306 display(OLED_RESET);

//Ultrasonic Setup///////////////////////////////////////
const int trigPinFront = 8;          //Connect US trig to arduino pin 9
const int echoPinFront = 9;         //Connect US echo to arduino pin 10
const int trigPinLeft = 48;          
const int echoPinLeft = 49;         
const int trigPinBack = 35;         
const int echoPinBack = 34;         
const int trigPinRight = 28;          
const int echoPinRight = 26;         

long duration;                  //Initialized variables
float distance;

//LiDAR Setup////////////////////////////////////////////
volatile float liDARvalright = 0;    //Initialized variables
volatile float liDARvalleft = 0;

//for IR input/output////////////////////////////////////
#include "IRLibAll.h"

IRrecvPCI myReceiver(2);    //Connect IRreceive pin (leftmost when facing you) to pin 2 on arduino
IRdecode myDecoder;         //Creates IRdecoder object
IRsend mySender;            //Creates IRsender object
int8_t codeProtocol;                //8bit int for code protocal (eg. NEC, SONY, UNKNOWN)
uint32_t RcodeValue;                //32bit int for received code
uint8_t RcodeBits;                  //8bit int for length of received code
String displaycode = "";            //empty string for OLED display of code
int code[] = {0, 0, 0, 0, 0, 0, 0, 0};

//Button Setup////////////////////////////////////////////
const int buttonfrontpin = 50;
const int buttonleftpin = 52;
const int buttonbackpin = 53;
const int buttonrightpin = 24;
const int buttontopleftpin = 40;
const int buttontoprightpin = 32;
int buttonfrontstate = 0;
int buttonleftstate = 0;
int buttonbackstate = 0;
int buttonrightstate = 0;
int buttontopleftstate = 0;
int buttontoprightstate = 0;

//MOTOR Setup////////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *motorFL = AFMS.getMotor(1);
Adafruit_DCMotor *motorFR = AFMS.getMotor(2);
Adafruit_DCMotor *motorBR = AFMS.getMotor(3);
Adafruit_DCMotor *motorBL = AFMS.getMotor(4);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
String IRRead(void) {
  if (myReceiver.getResults()) { 
      if ((recvGlobal.recvLength == 20) && 
          (recvGlobal.recvBuffer[1] - 10000 < 1000) && 
          (recvGlobal.recvBuffer[2] - 5200 < 500) && 
          (recvGlobal.recvBuffer[19] - 400 < 500)) {
        for (int i = 4, j = 0; i < 19; i +=2, j++) {
          if (abs(recvGlobal.recvBuffer[i] - 2000) < 500) {
            code[j] = 1;
          }
          else if (abs(recvGlobal.recvBuffer[i] - 400) < 500) {
            code[j] = 0;
          }
        }
        Serial.print("Code: ");
        for (int i = 0; i < 8; i++) {
          Serial.print(code[i]);
          Serial.print(", ");
        }
        Serial.print("\n\n");
        if (code[0] == 0 && code[1] == 0 && code[2] == 0 && code[3] == 0 && code[4] == 0) {
          if (code[5] == 0 && code[6] == 0 && code[7] == 0) {
            displaycode = "000";
          }
          else if (code[5] == 0 && code[6] == 0 && code[7] == 1) {
            displaycode = "001";
          }
          else if (code[5] == 0 && code[6] == 1 && code[7] == 0) {
            displaycode = "010";
          }
          else if (code[5] == 0 && code[6] == 1 && code[7] == 1) {
            displaycode = "011";
          }
          else if (code[5] == 1 && code[6] == 0 && code[7] == 0) {
            displaycode = "100";
          }
          else if (code[5] == 1 && code[6] == 0 && code[7] == 1) {
            displaycode = "101";
          }
          else if (code[5] == 1 && code[6] == 1 && code[7] == 0) {
            displaycode = "110";
          }
          else if (code[5] == 1 && code[6] == 1 && code[7] == 1) {
            displaycode = "111";
          }
          myReceiver.enableIRIn();
          return displaycode;
        }
        else if (code[0] == code[1] == code[2] == code[3] == code[4] == code[5] == code[6] == code[7] == 1) {
          displayText("In position");
          myReceiver.enableIRIn();
          return "null";
        }
        myReceiver.enableIRIn();
      }
      else {
        myReceiver.enableIRIn();
        return "null";
      }
      myReceiver.enableIRIn();      //Restart receiver
    }
    else {
      return "null";
    }
}

void displayText(String stringa) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(stringa);
  display.display();
}

void DisplayUltrasonic() {
 display.clearDisplay();         //Clears the display
   display.setCursor(0, 0);
  
   display.print("Front: ");  //Ultrasonic sensor
   display.print(UltrasonicRead("front"));
   display.print("\n");
   display.print("Back: ");  //Ultrasonic sensor
   display.print(UltrasonicRead("back"));
   display.print("\n");
   display.print("Left: ");  //Ultrasonic sensor
   display.print(UltrasonicRead("left"));
   display.print("\n");
   display.print("Right: ");  //Ultrasonic sensor
   display.print(UltrasonicRead("right"));
   display.display();
    Serial.print("Ultrasonic distance (right): ");
    Serial.println(distance);
}

/*void PrintUltrasonic() {
  Serial.print("Front: ");
}*/

void DisplayLidar() {
 display.clearDisplay();         //Clears the display
 display.setCursor(0, 0);

 display.print("1) ");           //First liDAR sensor
 display.print(liDARvalright);
 display.print("\n");

 display.print("2) ");           //Second liDAR sensor
 display.print(liDARvalleft);
 display.print("\n");

 display.display();              //Actually displays all of the above

 Serial.print("liDARvalright: ");
 Serial.println(liDARvalright);
 Serial.print("liDARvalleft: ");
 Serial.println(liDARvalleft);
}

float UltrasonicRead(String direction) {
 int trigPin = 0;
 int echoPin = 0;
 if (direction == "front") {
  trigPin = trigPinFront;
  echoPin = echoPinFront;
 }
 else if (direction == "back") {
  trigPin = trigPinBack;
  echoPin = echoPinBack;
 }
 else if (direction == "left") {
  trigPin = trigPinLeft;
  echoPin = echoPinLeft;
 }
 else if (direction == "right") {
  trigPin = trigPinRight;
  echoPin = echoPinRight;
 }
 digitalWrite(trigPin, LOW);           //Set to low for 2 ms
 delayMicroseconds(2);

 digitalWrite(trigPin, HIGH);          //Set to high for 10 ms, then back to low
 delayMicroseconds(10);
 digitalWrite(trigPin, LOW);

 duration = pulseIn(echoPin, HIGH);    //Save the duration as the length of the pulse where echopin reads high
 distance = duration * 0.034/2;        //distance is a function of the duration
 distance = distance * 0.393701;
 return distance;
}


void LidarRead() {
 //I just basically copied the below code from an example from somewhere
  while(Serial1.available()>=9)  {
      if((0x59 == Serial1.read()) && (0x59 == Serial1.read())) // byte 1 and byte 2
      {
        unsigned int t1 = Serial1.read(); // byte 3 = Dist_L
        unsigned int t2 = Serial1.read(); // byte 4 = Dist_H
        t2 <<= 8;
        t2 += t1;
        liDARvalright = t2;
        liDARvalright = liDARvalright * 0.393701;
        t1 = Serial1.read(); // byte 5 = Strength_L
        t2 = Serial1.read(); // byte 6 = Strength_H
        t2 <<= 8;
        t2 += t1;
        for(int i=0; i<3; i++)Serial1.read(); // byte 7, 8, 9 are ignored
      }
  }
  delay(50);

  while(Serial2.available()>=9) // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
  {
    if((0x59 == Serial2.read()) && (0x59 == Serial2.read())) // byte 1 and byte 2
    {
      unsigned int t1b = Serial2.read(); // byte 3 = Dist_L
      unsigned int t2b = Serial2.read(); // byte 4 = Dist_H
      t2b <<= 8;
      t2b += t1b;
      liDARvalleft = t2b;
      liDARvalleft = liDARvalleft * 0.393701;
      t1b = Serial2.read(); // byte 5 = Strength_L
      t2b = Serial2.read(); // byte 6 = Strength_H
      t2b <<= 8;
      t2b += t1b;
      for(int i=0; i<3; i++)Serial2.read(); // byte 7, 8, 9 are ignored
    }
  }
}

void Move_Forward(int spd) {
  motorFL->run(FORWARD);
  motorFR->run(FORWARD);
  motorBL->run(FORWARD);
  motorBR->run(FORWARD);
  motorFL->setSpeed(spd);
  motorFR->setSpeed(spd);
  motorBL->setSpeed(spd);
  motorBR->setSpeed(spd);
}

void Move_Backward(int spd) {
  motorFL->run(BACKWARD);
  motorFR->run(BACKWARD);
  motorBL->run(BACKWARD);
  motorBR->run(BACKWARD);
  motorFL->setSpeed(spd);
  motorFR->setSpeed(spd);
  motorBL->setSpeed(spd);
  motorBR->setSpeed(spd);
}

void Move_Left(int spd) {
  motorFL->run(BACKWARD);
  motorFR->run(FORWARD);
  motorBL->run(FORWARD);
  motorBR->run(BACKWARD);
  motorFL->setSpeed(spd);
  motorFR->setSpeed(spd);
  motorBL->setSpeed(spd);
  motorBR->setSpeed(spd);
}
void Move_Right(int spd) {
  motorFL->run(FORWARD);
  motorFR->run(BACKWARD);
  motorBL->run(BACKWARD);
  motorBR->run(FORWARD);
  motorFL->setSpeed(spd);
  motorFR->setSpeed(spd);
  motorBL->setSpeed(spd);
  motorBR->setSpeed(spd);
}

void Move_Stop() {
  motorFL->setSpeed(0);
  motorFR->setSpeed(0);
  motorBL->setSpeed(0);
  motorBR->setSpeed(0);
  motorFL->run(RELEASE);
  motorFR->run(RELEASE);
  motorBL->run(RELEASE);
  motorBR->run(RELEASE);
}

//State Machine///////////////////////////////////////////////////////////////////////////////////

typedef enum {
  STATE_IR_READ,
  STATE_A_BUTTON,
  STATE_A_CENTER,
  STATE_RAMP_DOWN,
  STATE_B_BUTTON,
  STATE_TREASURE,
  STATE_B_CENTER,
  STATE_RAMP_UP,
  STATE_C_BUTTON
} state_t;

String state_names[] = {
  "STATE_IR_READ - Read and display IR code",
  "STATE_A_BUTTON - Press correct button A",
  "STATE_A_CENTER - Center in upper area",
  "STATE_RAMP_DOWN - Move down ramp",
  "STATE_B_BUTTON - Press correct pressure plate B",
  "STATE_TREASURE - Push treasure box",
  "STATE_B_CENTER - Center in lower area",
  "STATE_RAMP_UP - Move up ramp",
  "STATE_C_BUTTON - Press correct button C"
};

void print_state(String e_state_string) {
  static String e_last_state = "null";
  if (e_state_string != e_last_state) {
    e_last_state = e_state_string;
    Serial.println(e_state_string);
  }
}

void update_state(void) {
  static state_t e_state = STATE_IR_READ;
  
  //IR_Read
  static bool irreadflag = 0;
  static int irreadnumber = 0;
  static String irreads[] = {"null","null","null","null"};
  static String irreadcurrent = "null";
  static int coursecode = 0;
  
  //A_Center
  float centermargin = 3.0;

  //Ramp
  float coursewidth = 35.0;

  //B_Button
  float bbuttondistance = 37.0;
  float bmargin = 5.0;
  
  switch(e_state) {
    case STATE_IR_READ:
      displayText("Scanning for IR code");
      irreadcurrent = IRRead();
      if (irreadcurrent != "null" && irreadcurrent != "") {
        irreads[irreadnumber] = irreadcurrent;
        irreadnumber++;
        if (irreadnumber == 4) {
          irreadnumber = 0;
        }
        Serial.print("irreads: [");
        for (int i = 0; i < 3; i++) {
          Serial.print("\"");
          Serial.print(irreads[i]);
          Serial.print("\"");
          Serial.print(", ");
        }
        Serial.print("\"");
        Serial.print(irreads[3]);
        Serial.print("\"]");
        Serial.print("\n");
      }
      if ((irreads[0] != "null" && irreads[0] != "11111111" && irreads[0] == irreads[1]) ||
          (irreads[0] != "null" && irreads[0] != "11111111" && irreads[0] == irreads[2]) ||
          (irreads[0] != "null" && irreads[0] != "11111111" && irreads[0] == irreads[3]) ||
          (irreads[1] != "null" && irreads[1] != "11111111" && irreads[1] == irreads[2]) ||
          (irreads[1] != "null" && irreads[1] != "11111111" && irreads[1] == irreads[3]) ||
          (irreads[2] != "null" && irreads[2] != "11111111" && irreads[2] == irreads[3])) {
        irreadflag = 1;
        Serial.println("Flag set");
      }
      if (irreadflag == 1) {
        Serial.println("Deciding which code");
        irreadflag = 0;
        if (irreadcurrent == "000") {
          coursecode = 1;
        }
        else if (irreadcurrent == "001") {
          coursecode = 2;
        }
        else if (irreadcurrent == "010") {
          coursecode = 3;
        }
        else if (irreadcurrent == "011") {
          coursecode = 4;
        }
        else if (irreadcurrent == "100") {
          coursecode = 5;
        }
        else if (irreadcurrent == "101") {
          coursecode = 6;
        }
        else if (irreadcurrent == "110") {
          coursecode = 7;
        }
        else if (irreadcurrent == "111") {
          coursecode = 8;
        }
        display.clearDisplay();
        display.setTextSize(4);
        display.setCursor(0, 0);
        display.print((String) coursecode);
        display.display();
        //delay(2000);
        e_state = STATE_A_BUTTON;
        Serial.println(state_names[e_state]);
        //Serial.println("moving on");
        //displayText(state_names[e_state]);
        //delay(2000);
        /*for (int i = 0; i < 4; i++){
          irreads[i] = "null";
        }*/
      }
      break;
    case STATE_A_BUTTON:
      if (digitalRead(buttonbackpin) == LOW) {
        Move_Backward(255);
        Serial.println("Moving backward");
        //displayText("Moving backward");
      }
      else {
        Move_Stop();
        if (displaycode[0] == '0') {
          if (digitalRead(buttontopleftpin) == LOW) {
            Move_Left(255);
            Serial.println("Moving left");
          }
          else {
            Move_Stop();
            e_state = STATE_A_CENTER;
            Serial.println(state_names[e_state]);
            Move_Right(255);
            delay(6000);
            Move_Stop();
          }
        }
        else {
          if (digitalRead(buttontoprightpin) == LOW) {
            Move_Right(255);
            Serial.println("Moving right");
          }
          else {
            Move_Stop();
            e_state = STATE_A_CENTER;
            Serial.println(state_names[e_state]);
            Move_Left(255);
            delay(6000);
            Move_Stop();
          }
        }
      }
      break;
    case STATE_A_CENTER:
      if (digitalRead(buttonbackpin) == LOW) {
        Move_Backward(200);
        Serial.println("Moving backward");
      }
      else {
        Move_Stop();
        Move_Forward(255);
        delay(1000);
        Move_Stop();
        //PrintUltrasonic();
        if ((UltrasonicRead("left") - UltrasonicRead("right")) > centermargin) {
          Move_Left(255);
          Serial.println("US not even, moving left");
          Serial.print("Left: ");
          Serial.println(UltrasonicRead("left"));
          Serial.print("Right: ");
          Serial.println(UltrasonicRead("right"));
        }
        else if ((UltrasonicRead("right") - UltrasonicRead("left")) > centermargin) {
          Move_Right(255);
          Serial.println("US not even, moving right");
        }
        else {
          Move_Stop();
          e_state = STATE_RAMP_DOWN;
          Serial.println(state_names[e_state]);
          delay(2000);
          Serial.println("Moving forward");
          Move_Forward(255);
          delay(1000);
          Serial.println("Tilting");
          Move_Stop();
          motorFR->run(FORWARD);
          motorBR->run(FORWARD);
          motorFR->setSpeed(255);
          motorBR->setSpeed(255);
          delay(800);
          Serial.println("Moving down the ramp");
          Move_Forward(255);
          delay(5000);
          Move_Stop();
        }
      }
      break;
    case STATE_RAMP_DOWN:
      Move_Forward(255);
      Serial.println("Moving forward");
      if (UltrasonicRead("left") + UltrasonicRead("right") < coursewidth) {
        Serial.println("Ultrasonics say we are down");
        e_state = STATE_B_BUTTON;
        Serial.println(state_names[e_state]);
        Move_Stop();
      }
      break;
    case STATE_B_BUTTON:
    /*
      if ((UltrasonicRead("front") - bbuttondistance) > bmargin) {
        Move_Forward(255);
      }
      else {
        Move_Stop();
        if (UltrasonicRead("left") > 2) {
          Move_Left(255);
        }
        else {
          Move_Stop();
          Move_Right(255);
          if (UltrasonicRead("right") - 18.0 > 3.0) {
            Move_Stop();
            e_state = STATE_TREASURE;
            displayText(state_names[e_state]);
          }
        }
      }*/
      if (displaycode[1] == '1') {
        Move_Forward(255);
        Serial.println("Forward 3 seconds");
        delay(3000);
        Move_Right(255);
        Serial.println("Right 5s");
        delay(5000);
        e_state = STATE_TREASURE;
        Serial.println(state_names[e_state]);
        Move_Left(255);
        Serial.println("Left 2s");
        delay(2000);
        Move_Forward(255);
        Serial.println("Forward 2s");
        delay(2000);
        Move_Right(255);
        Serial.println("Right 7s");
        delay(7000);
        Move_Left(255);
        Serial.println("Left 4s");
        delay(4000);
        Move_Stop();
      }
      else if (displaycode[1] == '0') {
        Move_Forward(255);
        Serial.println("Forward 3 seconds");
        delay(3000);
        Move_Left(255);
        Serial.println("Right 5s");
        delay(5000);
        e_state = STATE_TREASURE;
        Serial.println(state_names[e_state]);
        Move_Right(255);
        Serial.println("Left 2s");
        delay(2000);
        Move_Forward(255);
        Serial.println("Forward 2s");
        delay(2000);
        Move_Left(255);
        Serial.println("Right 7s");
        delay(7000);
        Move_Right(255);
        Serial.println("Left 4s");
        delay(4000);
        Move_Stop();
      }
      break;
    case STATE_TREASURE:
      /*if (UltrasonicRead("right") - UltrasonicRead("left") > 3.0) {
        Move_Right(255);
      }
      else if (UltrasonicRead("left") - UltrasonicRead("right") > 3.0) {
        Move_Left(255);
      }
      else {
        Move_Forward(255);
        delay(2000);
        Move_Stop();
        e_state = STATE_B_CENTER;
        //displayText(state_names[e_state]);
      }*/
      Move_Forward(255);
      Serial.println("Forward 4s");
      delay(4000);
      Move_Left(255);
      Serial.println("left 1s");
      delay(1000);
      Move_Backward(255);
      Serial.println("backward 2s");
      delay(2000);
      Move_Right(255);
      Serial.println("Right 1s");
      delay(1000);
      break;
    case STATE_B_CENTER:
      //if (UltrasonicRead("left")f
      /*
      while ((UltrasonicRead("left") - UltrasonicRead("right")) > centermargin) {
        Move_Left();
      }
      while ((UltrasonicRead("right") - UltrasonicRead("left")) > centermargin) {
        Move_Right();
      }
      LidarRead();
      while ((liDARvalright - liDARvalleft) > centermargin) {
        Move_Left();
        LidarRead();
      }
      LidarRead();
      while ((liDARvalleft - liDARvalright) > centermargin) {
        Move_Right();
        LidarRead();
      }
      e_state = STATE_RAMP_UP;
    */
      e_state = STATE_RAMP_UP;
      Serial.println(state_names[e_state]);
      //displayText(state_names[e_state]);
      break;
    case STATE_RAMP_UP:
      Move_Backward(255);
      displayText("Backward 6s");
      delay(6000);
      e_state = STATE_C_BUTTON;
      Serial.println(state_names[e_state]);/*
      if (UltrasonicRead("left") < coursewidth && UltrasonicRead("right") < coursewidth) {
        e_state = STATE_C_BUTTON;
      }
      if (digitalRead(8) == LOW) {
        e_state = STATE_C_BUTTON;
        displayText(state_names[e_state]);
      }*/
      break;
    case STATE_C_BUTTON:
      if (digitalRead(8) == HIGH) {
        e_state = STATE_IR_READ;
        Serial.println(state_names[e_state]);
        //displayText(state_names[e_state]);
      }
      break;
    default:
      e_state = STATE_IR_READ;
  }
  print_state(state_names[e_state]);
}

void setup() {
    Serial.begin(9600);          //Serial for IR and ultrasonic
    Serial1.begin(115200);      //HW Serial for TFmini
    Serial2.begin(115200);      //For second TFmini
    delay (2000);               //Give a little time for things to start
    
    //Serial.println("Sending start code:");
    
    // Set to Standard Output mode
    Serial1.write(0x42);
    Serial1.write(0x57);
    Serial1.write(0x02);
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x01);
    Serial1.write(0x06);

    Serial2.write(0x42);
    Serial2.write(0x57);
    Serial2.write(0x02);
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x01);
    Serial2.write(0x06);

    ///Display setup
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);      //0x3C might have to change
    delay(2000);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);


    ///Ultrasonic setup
    pinMode(trigPinFront, OUTPUT);
    pinMode(echoPinFront, INPUT);
    pinMode(trigPinBack, OUTPUT);
    pinMode(echoPinBack, INPUT);
    pinMode(trigPinLeft, OUTPUT);
    pinMode(echoPinLeft, INPUT);
    pinMode(trigPinRight, OUTPUT);
    pinMode(echoPinRight, INPUT);
    
    //IR Receiver Setup
    myReceiver.enableIRIn();                                //Enable receiver

    pinMode(buttonfrontpin, INPUT_PULLUP);
    pinMode(buttonrightpin, INPUT_PULLUP);
    pinMode(buttonleftpin, INPUT_PULLUP);
    pinMode(buttonbackpin, INPUT_PULLUP);
    pinMode(buttontopleftpin, INPUT_PULLUP);
    pinMode(buttontoprightpin, INPUT_PULLUP);

    //MOTORS
    AFMS.begin();  // create with the default frequency 1.6KHz
    motorFR->setSpeed(0);
    motorFL->setSpeed(0);
  
    motorBL->setSpeed(0);
    motorBR->setSpeed(0);
    
    motorFL->run(FORWARD);
    motorFR->run(FORWARD);
    motorBL->run(FORWARD);
    motorBR->run(FORWARD);
    
    motorFL->run(RELEASE);
    motorFR->run(RELEASE);
    motorBL->run(RELEASE);
    motorBR->run(RELEASE);
    Move_Stop();
}

void loop() {
  //////LiDAR Sensors////////////////////////////////////////////////////////////////////////////////////////
  delay(100);               // Don't want to read too often as TFmini samples at 100Hz


  //There need to be states where it is reading certain things; it cannot read from the IR and the sensors simultaneously really
  /*LidarRead();
  Serial.print("liDARvalright: ");
  Serial.println(liDARvalright);
  Serial.print("liDARvalleft: ");
  Serial.println(liDARvalleft);*/
  //displayText((String) UltrasonicRead("back"));
  //DisplayLidarUltrasonic();
  //IRRead();                       //Calls IR Read function (defined below)
  update_state();
  /*Move_Forward(255);
  delay(1000);
  Move_Stop();
  DisplayUltrasonic();
  delay(5000);*/
  /*
   delay(500);
   display.clearDisplay();         //Clears the display
   display.setCursor(0, 0);
  
   display.print("Front: ");  //Ultrasonic sensor
   display.print(UltrasonicRead("front"));
   display.print("\n");
   display.print("Back: ");  //Ultrasonic sensor
   display.print(UltrasonicRead("back"));
   display.print("\n");
   display.print("Left: ");  //Ultrasonic sensor
   display.print(UltrasonicRead("left"));
   display.print("\n");
   display.print("Right: ");  //Ultrasonic sensor
   display.print(UltrasonicRead("right"));
   display.display();
*//*
   while (digitalRead(buttonleftpin) == HIGH) {
    Move_Right(255);
   }
   Move_Stop();
   while (digitalRead(buttonrightpin) == HIGH) {
    Move_Left(255);
   }
   Move_Stop();
   while (digitalRead(buttonfrontpin) == HIGH) {
    Move_Backward(255);
   }
   Move_Stop();
   while (digitalRead(buttonbackpin) == HIGH) {
    Move_Forward(255);
   }
   Move_Stop();*/
   
}


