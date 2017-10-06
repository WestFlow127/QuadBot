#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#include <math.h>
#define NO_SOUND 0 // make the rests in music

#define PIN            5
#define NUMPIXELS      8

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);  
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);
    
  long xA = -5;
  long xB = -5;
  long xC = 5;
  long xD = 5;

  int ages = 10000;
 
const int pingPin = 7;

int timee = 200;

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

void loop()
{
    delay(25);
    
    //CurlUp();
    
    for(int i=0;i<30;i++){
      Set_Body_Centre(60,i,i,-120);
      Set_Neopixels(0,map(i,0,30,0,255),0);
      delay(5);
    } 
    
    delay(25);

    Walk_Forwards(6,2,-60);
    delay(15);
    Set_j2('D',map(15,0,200,100,80));
    Set_j1('D',map(15,0,200,0,80));
    delay(15); 
    wave(200,1);
    wave(200,1);
    wave(200,1);
    
    bounce(2);
    
    Rotate_Clockwise(9, 2);

    delay(5);
    
    Walk_Forwards(2,2,-60);
    
    for( int i = 30; i > 0; i-- ){
      Set_Body_Centre(60, i, i, -120);
      Set_Neopixels(0, map(i, 30, 0, 255, 0), 0);
      delay(5);
    }
  
    Set_Neopixels(200,0,255);
    
    CurlUp();
    
    while(1){
      rainbowCycle(10);
    }
}

void bounce( int iterator ){
  for(int i = 0; i < iterator; i++){
    Set_Body_Centre(60,0,0,-120);
    delay(250);
    Set_Body_Centre(60,0,0,-100);
    delay(250);
    Set_Body_Centre(60,0,0,-120);
    delay(250);
    Set_Body_Centre(60,0,0,-100);
    delay(250);
    Set_Body_Centre(60,0,0,-120);
    Set_Body_Centre(60,0,0,-120);
    delay(250);
    Set_Body_Centre(60,0,0,-100);
    delay(250);
    Set_Body_Centre(60,0,0,-120);
    delay(250);
    Set_Body_Centre(60,0,0,-100);
    delay(250);
    Set_Body_Centre(60,0,0,-120);
  }
}

void wave(int iterator, int dlay){
 
  for(int i=0;i<iterator;i++){
    Set_Neopixels(255,200,0);
    Set_j0('B',map(i,0,iterator,100,80));
    if(i<iterator/2){
      Set_j2('B',map(i,0,iterator/2,160,140));
      Set_j1('B',map(i,0,iterator/2,30,20));
    }
    else{
      Set_j2('B',map(i,iterator/2,iterator,140,160));
      Set_j1('B',map(i,iterator/2,iterator,20,30));
    }
    delay(dlay);
  }  
  
  for(int i=0;i<iterator;i++){
    Set_Neopixels(0,255,0);
    Set_j0('B',map(i,0,iterator,80,100));
    if(i<iterator/2){
      Set_j2('B',map(i,0,iterator/2,160,140));
      Set_j1('B',map(i,0,iterator/2,30,20));
    }
    else{
      Set_j2('B',map(i,iterator/2,iterator,140,160));
      Set_j1('B',map(i,iterator/2,iterator,20,30));
    }   
    delay(dlay);
  }

}

void CurlUp(){
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
