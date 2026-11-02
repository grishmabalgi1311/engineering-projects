// #include <Servo.h>
// // #include <SoftwareSerial.h>

// // #define SOLENOID_CONTROL 13 
// // #define BUTTON_CCW 6
// // #define BUTTON_CW 7

// #define SERVO_PIN 2 

// // #define PROP_RX_PIN 10   // Arduino RX <- Prop TX
// // #define PROP_TX_PIN 11   // Arduino TX -> Prop RX (through divider)
// // #define PROP_BAUD   9600

// static const int SERVO_STOP = 90;
// static const int SERVO_CW   = 130;
// static const int SERVO_CCW  = 50;b

// // SoftwareSerial prop(PROP_RX_PIN, PROP_TX_PIN);
// Servo pivot;

// static inline void stop_pivot() { pivot.write(SERVO_STOP); }
// static inline void run_CW()     { pivot.write(SERVO_CW); }
// static inline void run_CCW()    { pivot.write(SERVO_CCW); }

// void solenoid_begin(){
//   digitalWrite(SOLENOID_CONTROL,1-digitalRead(SOLENOID_BUTTON));
// }

// void solenoid_end(){
//   digitalWrite(SOLENOID_CONTROL, LOW);
// }

// void setup()
// {
//   // pinMode(SOLENOID_CONTROL,OUTPUT);
//   pinMode(BUTTON_CW, INPUT_PULLUP);
//   pinMode(BUTTON_CCW, INPUT_PULLUP);

//   pivot.attach(SERVO_PIN);
//   stop_pivot();

//   // prop.begin(PROP_BAUD);
// }

// void loop() {

//   // if (!prop.available()) return;

//   // char cmd = (char)prop.read();
//   // if (cmd != 'S') return;

//   // while (prop.available()) prop.read(); //clear buffer

//   // stop_pivot();

//   // solenoid_begin();

//   // while (true) {
//   bool cwPressed  = (digitalRead(BUTTON_CW)  == LOW);
//   bool ccwPressed = (digitalRead(BUTTON_CCW) == LOW);

//   if (cwPressed && !ccwPressed) {
//     pivot.write(SERVO_CW);
//   }
//   else if (ccwPressed && !cwPressed) {
//     pivot.write(SERVO_CCW);
//   }
//   else {
//     pivot.write(SERVO_STOP); //none or both pressed
//   }
//   // }

//   // solenoid_end();

//   pivot.write(SERVO_STOP);
//   // prop.write('D');  // notify propeller pivot complete

// }

#include <Servo.h>
// #include <SoftwareSerial.h>

// #define SOLENOID_CONTROL 13 
// #define BUTTON_CCW 6
// #define BUTTON_CW 7

#define SERVO_PIN 2 

// #define PROP_RX_PIN 10   // Arduino RX <- Prop TX
// #define PROP_TX_PIN 11   // Arduino TX -> Prop RX (through divider)
// #define PROP_BAUD   9600

static const int SERVO_STOP = 0;//90;
static const int SERVO_CW   = 130;
static const int SERVO_CCW  = 50;

// SoftwareSerial prop(PROP_RX_PIN, PROP_TX_PIN);
Servo pivot;

const int BTN1 = 6;
const int BTN2 = 7;

bool btn1;
bool btn2;

void setup() {
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);

  pivot.attach(SERVO_PIN);

  pivot.write(SERVO_STOP);
}

void loop() {

  btn1 = digitalRead(BTN1);  // HIGH = not pressed, LOW = pressed
  btn2 = digitalRead(BTN2);

  if (!btn1 && btn2) {
    pivot.write(SERVO_CW);
    // delay(300);
  } 
  else if (btn1 && !btn2) {
    pivot.write(SERVO_CCW);
    // delay(300);
  }
  else if (!btn1 && !btn2) {
    pivot.write(SERVO_STOP);
    // delay(300);
  }
  else{
    pivot.write(SERVO_STOP);
  }
}