 #include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#include <math.h>

#define PIN            5
#define NUMPIXELS      8

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);  
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);
    

void setup() {
  Serial.begin(9600);
  
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
  pixels.begin();
  yield();
  delay(500);
  strip.begin();
  strip.show();
}


void loop() {     

  Set_Neopixels(0,255,0);
  delay(500);
  Set_Neopixels(0,0,0);
  delay(500);
  Set_Neopixels(0,255,0);
  
  Set_j0('A', 45);
  Set_j0('B', 45);
  Set_j0('C', 45);
  Set_j0('D', 45);
  
  Set_j1('A', 0);
  Set_j1('B', 0);
  Set_j1('C', 0);
  Set_j1('D', 0);
  
  
  Set_j2('A', 0);
  Set_j2('B', 0);
  Set_j2('C', 0);
  Set_j2('D', 0);
}


//CORE FUNCTIONS
void Set_j0(char leg, int angle){
  //Set the angle of joint j0, in degrees
  //Constrain the angle to safe limits
  angle = constrain(angle, -10, 100);

  //Choose the right PWM pin based on which leg
  int pin = 0;
  switch(leg){
    case 'A':
      pin = 0;
      break;

    case 'B':
      pin = 3;
      break;

    case 'C':
      pin = 8;
      break;

    case 'D':
      pin = 11;
      break;

    default:
      break;
  }
  
  pwm.setPWM(pin, 0, map(angle, 0, 90, 196, 390));
}

void Set_j1(char leg, int angle){
  //Set the angle of joint j0, in degrees
  //Constrain the angle to safe limits
  angle = constrain(angle, 0, 140);

 //Choose the right PWM pin based on which leg
  int pin = 1;
  switch(leg){
    case 'A':
      pin = 1;
      break;

    case 'B':
      pin = 4;
      break;

    case 'C':
      pin = 9;
      break;

    case 'D':
      pin = 12;
      break;

    default:
      break;
  }
  
  pwm.setPWM(pin, 0, map(angle, 0, 90, 110, 294));
}

void Set_j2(char leg, int angle){
  //Set the angle of joint j0, in degrees
  //Constrain the angle to safe limits
  angle = constrain(angle, 0, 180);

 //Choose the right PWM pin based on which leg
  int pin = 2;
  switch(leg){
    case 'A':
      pin = 2;
      break;

    case 'B':
      pin = 5;
      break;

    case 'C':
      pin = 10;
      break;

    case 'D':
      pin = 13;
      break;

    default:
      break;
  }
  
  pwm.setPWM(pin, 0, map(angle, 0, 180, 110, 470));
}

void Set_Neopixels(int R, int G, int B){

  pixels.setPixelColor(0, pixels.Color(R,G,B));
  pixels.setPixelColor(1, pixels.Color(R,G,B));
  pixels.setPixelColor(2, pixels.Color(R,G,B));
  pixels.setPixelColor(3, pixels.Color(R,G,B));
  pixels.setPixelColor(4, pixels.Color(R,G,B));
  pixels.setPixelColor(5, pixels.Color(R,G,B));
  pixels.setPixelColor(6, pixels.Color(R,G,B));
  pixels.setPixelColor(7, pixels.Color(R,G,B));
  pixels.show();
}

