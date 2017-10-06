#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define PIN            5
#define NUMPIXELS      8

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);  
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);

SoftwareSerial mySerial(9, 10);


void setup() {
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
  pixels.begin();
  yield();
  strip.begin();
  strip.show();
  rainbowCycle(1);
  Set_Body_Centre(50, 0,0,-120);
}

void loop() { 
  Walk_Forwards(6,2,-80);
  wave(200, 1);
  wave(200, 1);
}



