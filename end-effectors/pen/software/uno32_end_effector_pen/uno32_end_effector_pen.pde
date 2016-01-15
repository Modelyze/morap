/*
Code for the end-effector controlled by an uno32
*/

#include <Servo.h>
#include <Wire.h>

// On-board leds
#define LED1 13
#define LED2 43

// Input
#define BUT1 2
#define BUT2 3
#define POT A0

// Servo definitions
#define SERVO_PIN 9
#define SERVO_RETRACTED 45 // 0 - 180
#define SERVO_DEPLOYED 8 // 0 - 180
#define SERVO_MEAN ((SERVO_DEPLOYED + SERVO_RETRACTED)/2)

// I2C stuff
#define I2C_ADDRESS 0b0011001
#define I2C_DEPLOY_SERVO 1
#define I2C_RETRACT_SERVO 2
#define I2C_SET_SERVO_POSITION 7

// Servo object
Servo myservo;

// variables
unsigned int sensorValue = 0, toServo = SERVO_RETRACTED;

void setup() {
  // Serial
  Serial.begin(9600);
  
  // I/O
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(BUT1,INPUT);
  pinMode(BUT2,INPUT);
  pinMode(POT,INPUT);
  
  // Servo attach
  myservo.attach(SERVO_PIN);
  myservo.write(SERVO_RETRACTED);
  
  // I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
}

void loop() {
  
  //ret_to_dep(); // testing
  
  // Manual pen control through buttons  
  if(digitalRead(BUT1) == HIGH) {
    myservo.write(SERVO_DEPLOYED);
    digitalWrite(LED2,HIGH);
  } else if (digitalRead(BUT2) == HIGH) {
    myservo.write(SERVO_RETRACTED);
    digitalWrite(LED2,LOW);
  }
  delay(50);
}


void receiveEvent(int n_bytes) {  
  static unsigned char islit = 0;
  
  // Blink led for each transmission
  if (!islit) {
    digitalWrite(LED1,HIGH);
    islit = 1;
  } else {
    digitalWrite(LED1,LOW);
    islit = 0;
  }
  
  if (Wire.available() == 0) {
    return; // Safeguard
  }
  
  unsigned char command = Wire.read();
  int set_value = SERVO_RETRACTED;
  
  
  switch(command) {
    case I2C_DEPLOY_SERVO:
      set_value = SERVO_DEPLOYED;
      break;
    case I2C_RETRACT_SERVO:
      set_value = SERVO_RETRACTED;
      break;
    case I2C_SET_SERVO_POSITION:
      delay(10); // Wait for next byte to arrive
      if(Wire.available() > 0) {
        set_value = Wire.read();
        set_value = map(set_value,0,255,SERVO_RETRACTED,SERVO_DEPLOYED);
      } else {
        set_value = SERVO_RETRACTED;
      }
      break;
    default:
      set_value = SERVO_RETRACTED;
  }
  
  myservo.write(set_value);
  Serial.println(set_value);
  
  // switch on/off led depending on pen position
  if ( abs(SERVO_DEPLOYED - set_value) < abs(SERVO_RETRACTED - set_value) ) {
    digitalWrite(LED2,HIGH);
  } else {
    digitalWrite(LED2,LOW);
  }
  
  while(Wire.available() > 0); // Clear buffer
  
  delay(10); // Slight delay
}


void ret_to_dep() {
  // Test map
  unsigned char p, set_value;
  
  for(p = 0; p <= 255; p++) {
    set_value = map(p,0,255,SERVO_RETRACTED,SERVO_DEPLOYED);
    myservo.write(set_value);
    // switch on/off led depending on pen position
    if ( abs(SERVO_DEPLOYED - set_value) < abs(SERVO_RETRACTED - set_value) ) {
      digitalWrite(LED2,HIGH);
    } else {
      digitalWrite(LED2,LOW);
    }
    delay(39);
  }
  
}

