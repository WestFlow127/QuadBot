
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

void Set_Leg_Position_Cartes(char whichLeg, double x, double y, double z){
  /*
  WHAT IS THIS FUNCTION?
  This function will move the endpoint of a robot leg to the specified
  xyz position.
  
  HOW TO USE THIS FUNCTION
  Specify which leg you would like to move by passing in either 'A', 'B'
  'C' or 'D'. Then pass in three values for x, y and z. The position is
  specified in millimeters, and is relative to the first joint on the
  hip of the robot. A good test position is x = 72/110, y = 0, z = 0

  Note that this function works in leg cartesian. That is, the cartesian
  system is specific for each leg, rather than being coherent with a wider
  reference frame (ie, the body). This is good for symmetrical movements 
  of the legs but not good for co-ordinated movements of the body. For
  co-ordinated body movement, use the Set_Leg_Position_OCartes() function.
  */
  
  //MATH CONSTANTS
  double pi = 3.14159265;
  
  //LEG CONSTANTS
  //Here we define the length, in mm, of each part of the leg.
  //If using a different leg, feel free to vary these constants.
  //Cox relates to the hip length.
  //Fem relates to the knee length.
  //Tib relates to the foot length.
  double cox = 41;
  double fem = 53;
  double tib = 94;
  //-----------------------------------------------------------
  
  //INTERMEDIATES
  //These intermediate variables are used in the IK calculation.
  double c, xp, yp, thetai, alpha, r, test;
  //-----------------------------------------------------------
  
  //INPUT VARIABLES
//  double x = 72;
//  double y = 0;
//  double z = 0;

  //OUTPUT VARIABLES
  double femangle;
  double tibangle;
  double coxangle;
  
  //IK CALCULATION
  //In this block of calculations, the three desired cartesian
  //co-ordinates are translated into three joint angles.
  
  y = -y;
  r = pow((sq(x)+sq(y)), 0.5);
  coxangle = ((asin(y/r)*0.8)/pi)*180;
  r = r-cox, z = z;
  test = r;
  thetai = atan2(z,r);
  xp = r - 1;
  yp = z; 
  c = ((sq(xp))+(sq(yp))-(sq(fem))-sq(tib))/(2*fem*tib);
  alpha = acos(-c);
  tibangle=((alpha+pi)/pi)*180;
  femangle =  ((atan2(yp,xp) + asin(tib*sin(alpha)/pow((sq(xp)+sq(yp)), (0.5))))/pi)*180;
  tibangle = tibangle - 360; 
  coxangle = coxangle + 45;
  femangle = -(femangle) + 90;
  tibangle = -(tibangle) + 90; 
  //-----------------------------------------------------------
  
  //WRITE ANGLES
  //Here the joint angles are written to the leg motors.
  //This concludes the IK process.
  Set_j0(whichLeg, coxangle);
  Set_j1(whichLeg, femangle);
  Set_j2(whichLeg, tibangle);
}

void Set_Leg_Position_OCartes(char whichLeg, double xb, double yb, double zb){

  //MATH CONSTANTS
    double one_over_root2 = 0.707106781;
  
//    //INPUT VARIABLES
//    char whichLeg = 'A';
//    double xb = 90;              //Ax-, Ay+: Bx-, By-: Cx+, Cy-: Dx+, Dy+
//    double yb = 90;
//    double zb = 0;
  
    //OUTPUT VARIABLES
    double xl;
    double yl;
    double zl;
  
    switch(whichLeg){
      case 'A':
        //LEG A ROTATION
        xl = yb*one_over_root2 - xb*one_over_root2;
        yl = xb*one_over_root2 + yb*one_over_root2;
        break;
  
      case 'B':
        //LEG B ROTATION
        yl = -xb*one_over_root2 + yb*one_over_root2;
        xl =  yb*one_over_root2 + xb*one_over_root2;
        break;
  
      case 'C':
       //LEG C ROTATION 
       yl = -yb*one_over_root2 - xb*one_over_root2;
       xl =  xb*one_over_root2 - yb*one_over_root2;
       break;
  
      case 'D':
       //LEG D ROTATION    
       xl =  xb*one_over_root2 + yb*one_over_root2;
       yl =  -yb*one_over_root2 + xb*one_over_root2;
       
       break;
    
      default:
        break;
    }
    
    zl = zb;
    Set_Leg_Position_Cartes(whichLeg, xl,yl,zl);
}

void Set_Body_Centre(float fan, long x_body_centre, long y_body_centre, long z_body_centre){
  
    for(byte i = 0; i < 4; i++){
      
  //  //INPUT VARIABLES
  //  float fan = 40;           //40
  //  long x_body_centre = 0;
  //  long y_body_centre = 0;
  //  long z_body_centre = -80; //-80
  
    //OUTPUT VARIABLES
    long xb;
    long yb;
    long zb = z_body_centre;
    
  
      switch(i){
      case 0:
        xb = -fan - x_body_centre;
        yb = fan - y_body_centre;
        Set_Leg_Position_OCartes('A', xb, yb, zb);
        break;
  
      case 1:
        xb = -fan - x_body_centre; 
        yb = -fan - y_body_centre;
        Set_Leg_Position_OCartes('B', xb, yb, zb);
        break;
  
      case 2:
        xb = fan - x_body_centre;
        yb = -fan - y_body_centre;
        Set_Leg_Position_OCartes('C', xb, yb, zb);
  
      case 3:
        xb = fan - x_body_centre;
        yb = fan - y_body_centre;
        Set_Leg_Position_OCartes('D', xb, yb, zb);
        
      default:
        break;
    }
      
    
    }
  }


//SPECIFIC MOVEMENT FUNCTIONS
void Fixed_Body_Circle_Move(){


//  for(int i=0; i<100; i=i+2){
//    Set_Body_Centre(40,0,0,map(i,0,100,0,-80)); 
//    delay(1);
//  }
//
//  delay(1000);
//
//  for(int i=0; i<100; i=i+2){
//    Set_Body_Centre(40,map(i,0,100,0,25),0,-80);
//    delay(1);
//  }

  
    double pi_over_180 = 0.0174532925199432; //pi/180
  
    Set_Body_Centre(40,25,0,-80);
   // delay(1000);
    
    for(int i=0; i<360; i=i+2){  
      double y = 25*sin(i*pi_over_180);
      double x = 25*cos(i*pi_over_180);
  
      Set_Body_Centre(40,x,y,-80);
  
      if(i<180)
        Set_Neopixels(i, 0, map(i,0,180,180,0));
  
      else
        Set_Neopixels(map(i,180,360,180,0), 0, map(i,180,360,0,180));
          
      delayMicroseconds(500);
    }
  }

void StandUp(){
  //fanout = 40
  //z=-80
  Set_Body_Centre(100, 0, 0, 0);
  delay(800);
  for(int i=0;i< 55; i++){
    Set_Body_Centre(100-i, 0, 0, map(i,0,55,0,-80));
    delay(5);
  }
}

void Step_Leg(char whichLeg, float Xstart, float Xend, float Ystart, float Yend, float Zdown, float Zup, int movedelay){

//  char whichLeg = 'A';
//  float Xstart = -40;
//  float Xend = -40;
//  float Ystart = 40;
//  float Yend = 0;
//  float Zdown = -80;
//  float Zup = -40;
//  int movedelay = 10;

  Set_Leg_Position_OCartes(whichLeg,Xstart,Ystart,Zdown);
  delay(10);

  float Xmove = Xend-Xstart;
  float Ymove = Yend-Ystart;

  for(byte i=0;i<50;i++){
  //STEP LEG (UP STROKE)
    if(i<25){
      Set_Leg_Position_OCartes(whichLeg,map(i,0,24,Xstart,Xstart+(Xmove/2)),map(i,0,24,Ystart,Ystart+(Ymove/2)),map(i,0,24,Zdown, Zup));
    }
  //STEP LEG (DOWN STROKE)
    else{
      Set_Leg_Position_OCartes(whichLeg,map(i,25,50,Xstart+(Xmove/2),Xend),map(i,25,50,Ystart+(Ymove/2),Yend),map(i,25,50,Zup,Zdown));
    }
    delay(movedelay);
  }
}



//WALKING FUNCTIONS


void WalkBackwards(float LegSpread, int UpDistance, int DownDistance, int StepDelay){
  long yA=30;
  long yB=-30;
  long yC=-30;
  long yD=30;
  int ages = 10000;
  for(int i = 0; i < 41; i++){
        yA=yA+1;
        yB=yB+1;
        yC=yC+1;
        yD=yD+1;
        
        Set_Leg_Position_OCartes('A', -LegSpread, yA, DownDistance);   
        Set_Leg_Position_OCartes('B', -LegSpread, yB, DownDistance);
        Set_Leg_Position_OCartes('C',  LegSpread, yC, DownDistance);
        Set_Leg_Position_OCartes('D',  LegSpread, yD, DownDistance);  
        delay(50);  
        
        if(i == 10){
          Set_Leg_Position_OCartes('A', -LegSpread, yA, DownDistance);
          delay(StepDelay);
          
          yA = 1;//45; //Best is 1 //Alt is 23
          Set_Leg_Position_OCartes('A', -LegSpread, yA, UpDistance);
          delay(StepDelay);
          
          
          Set_Leg_Position_OCartes('A', -LegSpread, yA, DownDistance);
          delay(StepDelay);
          
        }
        
        else if(i == 20){
          Set_Leg_Position_OCartes('D', LegSpread, yD, DownDistance);
          delay(StepDelay);
          
          yD = 1;  //Best is 1
          Set_Leg_Position_OCartes('D', LegSpread, yD, UpDistance);
          delay(StepDelay);
          
          Set_Leg_Position_OCartes('D', LegSpread, yD, DownDistance);
          delay(StepDelay);       
        }
        
        else if (i == 30){
          Set_Leg_Position_OCartes('C', LegSpread, yC, DownDistance);
          delay(StepDelay);
          
          yC = -30;
          Set_Leg_Position_OCartes('C', LegSpread, yC, UpDistance);
          delay(StepDelay);
          delay(ages);
          
          Set_Leg_Position_OCartes('C', LegSpread, yC, DownDistance);
          delay(StepDelay);
          
        }
        
        else if (i == 40){
          Set_Leg_Position_OCartes('B', -LegSpread, yB, DownDistance);
          delay(StepDelay);
          
          yB = -30;
          Set_Leg_Position_OCartes('B', -LegSpread, yB, DownDistance);
          delay(StepDelay);
          
          Set_Leg_Position_OCartes('B', -LegSpread, yB, UpDistance);
          delay(StepDelay);
        }
        
    
      }
}


void WalkLeft(float LegSpread, int UpDistance, int DownDistance, int StepDelay){

  for(int i = 0; i < 41; i++){
        //SETUP FROM STATIONARY
        xA++;
        xB++;
        xC++;
        xD++;
        
        if(i == 10){
          Set_Leg_Position_OCartes('C', xC, -LegSpread, DownDistance);
          delay(StepDelay);
          
          xC = 1;//45; //Best is 1 //Alt is 23
          Set_Leg_Position_OCartes('C', xC, -LegSpread, DownDistance);
          delay(StepDelay);
          
          Set_Leg_Position_OCartes('C', xC, -LegSpread, UpDistance);
          delay(StepDelay);
        }
        
        else if (i == 20){
          Set_Leg_Position_OCartes('D', xD, LegSpread, DownDistance);
          delay(StepDelay);
          
          xD = 1;  //Best is 1
          Set_Leg_Position_OCartes('D', xD, LegSpread, DownDistance);
          delay(StepDelay);
          
          Set_Leg_Position_OCartes('D', xD, LegSpread, UpDistance);
          delay(StepDelay);
        }
        
        else if (i == 30){
          Set_Leg_Position_OCartes('A', xA, LegSpread, DownDistance);
          delay(StepDelay);
          
          xA = -45;
          Set_Leg_Position_OCartes('A', xA, LegSpread, DownDistance);
          delay(StepDelay);
          
          Set_Leg_Position_OCartes('A', xA, LegSpread, UpDistance);
          delay(StepDelay);
        }
        
        else if (i == 40){
          Set_Leg_Position_OCartes('B', xB, -LegSpread, DownDistance);
          delay(StepDelay);
          
          xB = -45;
          Set_Leg_Position_OCartes('B', xB, -LegSpread, DownDistance);
          delay(StepDelay);
          
          Set_Leg_Position_OCartes('B', xB, -LegSpread, UpDistance);
          delay(StepDelay);
        }
        
        Set_Leg_Position_OCartes('A', xA, LegSpread, DownDistance);
        Set_Leg_Position_OCartes('B', xB, -LegSpread, DownDistance);
        Set_Leg_Position_OCartes('C', xC, -LegSpread, DownDistance); 
        Set_Leg_Position_OCartes('D', xD, LegSpread, DownDistance);  
      }
}


void WalkRight(float LegSpread, int UpDistance, int DownDistance, int StepDelay){
  for(int i = 0; i < 41; i++){
        //SETUP FROM STATIONARY
        xA--;
        xB--;
        xC--;
        xD--;
        
        if(i == 10){
          Set_Leg_Position_OCartes('A', xA, LegSpread, DownDistance);
          delay(StepDelay);
          
          xA = 1;//45; //Best is 1 //Alt is 23
          Set_Leg_Position_OCartes('A', xA, LegSpread, DownDistance);
          delay(StepDelay);
          
          Set_Leg_Position_OCartes('A', xA, LegSpread, UpDistance);
          delay(StepDelay);
        }
        
        else if (i == 20){
          Set_Leg_Position_OCartes('B', xB, -LegSpread, DownDistance);
          delay(StepDelay);
          
          xB = 1;  //Best is 1
          Set_Leg_Position_OCartes('B', xB, -LegSpread, DownDistance);
          delay(StepDelay);
          
          Set_Leg_Position_OCartes('B', xB, -LegSpread, UpDistance);
          delay(StepDelay);
        }
        
        else if (i == 30){
          Set_Leg_Position_OCartes('C', xC, -LegSpread, DownDistance);
          delay(StepDelay);
          
          xC = 45;
          Set_Leg_Position_OCartes('C', xC, -LegSpread, DownDistance);
          delay(StepDelay);
          
          Set_Leg_Position_OCartes('C', xC, -LegSpread, UpDistance);
          delay(StepDelay);
        }
        
        else if (i == 40){
          Set_Leg_Position_OCartes('D', xD, LegSpread, DownDistance);
          delay(StepDelay);
          
          xD = 45;
          Set_Leg_Position_OCartes('D', xD, LegSpread, DownDistance);
          delay(StepDelay);
          
          Set_Leg_Position_OCartes('D', xD, LegSpread, UpDistance);
          delay(StepDelay);
        }
        
        Set_Leg_Position_OCartes('A', xA, LegSpread, DownDistance);
        Set_Leg_Position_OCartes('B', xB, -LegSpread, DownDistance);    
        Set_Leg_Position_OCartes('C', xC, -LegSpread, DownDistance);
        Set_Leg_Position_OCartes('D', xD, LegSpread, DownDistance);  
    
      }
  
}


void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}



//SEPTEMBER WALKING FUNCTIONS

void StepY(char leg, int x, int ystart, int yend, int zup, int zdown, int delaytime){
  for(int i=0;i<21;i++){
    Set_Leg_Position_OCartes(leg, x,map(i,0,20,ystart,(yend/2)),map(i,0,20,zdown,zup));
    delay(delaytime);
  }
  for(int i=0;i<21;i++){
    Set_Leg_Position_OCartes(leg, x, map(i,0,20,(yend/2),yend),map(i,0,20,zup,zdown));
    delay(delaytime);
  }
}

void MoveY(int x, int yAstart, int yBstart, int yCstart, int yDstart, int body, int zdown, int dtime){
  for(int i=0;i<16;i++){
    Set_Leg_Position_OCartes('A', -x, map(i,0,15,yAstart, yAstart+body),zdown);
    Set_Leg_Position_OCartes('B', -x, map(i,0,15,yBstart,yBstart+body),zdown);
    Set_Leg_Position_OCartes('C', x, map(i,0,15,yCstart,yCstart+body),zdown);
    Set_Leg_Position_OCartes('D', x, map(i,0,15,yDstart,yDstart+body),zdown);
    delay(dtime);
  }
}

void StepXY(char leg, int xstart, int xend, int ystart, int yend, int zup, int zdown, int delaytime){
  for(int i=0;i<21;i++){
    Set_Leg_Position_OCartes(leg, map(i,0,20,xstart,(xend/2)),map(i,0,20,ystart,(yend/2)),map(i,0,20,zdown,zup));
    delay(delaytime);
  }
  for(int i=0;i<21;i++){
    Set_Leg_Position_OCartes(leg, map(i,0,20,(xend/2),xend), map(i,0,20,(yend/2),yend),map(i,0,20,zup,zdown));
    delay(delaytime);
  }
}

//////////////////////////////////////////////////////////

void StepX(char leg, int y, int xstart, int xend, int zup, int zdown, int delaytime){
  for(int i=0;i<21;i++){
    Set_Leg_Position_OCartes(leg, map(i,0,20,xstart,(xend/2)),y,map(i,0,20,zdown,zup));
    delay(delaytime);
  }
  for(int i=0;i<21;i++){
    Set_Leg_Position_OCartes(leg, map(i,0,20,(xend/2),xend),y,map(i,0,20,zup,zdown));
    delay(delaytime);
  }
}


void MoveX(int y, int xAstart, int xBstart, int xCstart, int xDstart, int body, int zdown, int dtime){
  for(int i=0;i<16;i++){
    Set_Leg_Position_OCartes('A', map(i,0,15,xAstart, xAstart+body),y ,zdown);
    Set_Leg_Position_OCartes('B', map(i,0,15,xBstart,xBstart+body),-y ,zdown);
    Set_Leg_Position_OCartes('C', map(i,0,15,xCstart,xCstart+body),-y ,zdown);
    Set_Leg_Position_OCartes('D', map(i,0,15,xDstart,xDstart+body),y ,zdown);
    delay(dtime);
  }
}
//////////////////////////////////////////////////////////

//RIGHT WALK
void PreWalkRight(int sdelay){
  StepX('A', 50, -50, 5, -80, -110, sdelay);
  StepX('D', 50, 50, -5, -80,-110, sdelay);
}

void WalkRightOnce(int sdelay, int mdelay){
    StepX('A', 50, 5, -105, -80, -120, sdelay);
    
    MoveX(50, -105, -50, 50, -5, 55, -110,mdelay);

    StepX('C',-50, 105, -5, -80, -110, sdelay);

    StepX('B', -50, 5, -105, -80, -110, sdelay);
    
    MoveX(50, -50, -105, -5, 50, 55, -110,mdelay);
  
    StepX('D', 50, 105, -5, -80, -110, sdelay);
}

void PostWalkRight(int sdelay){
  StepX('A', 50, 5, -50, -80, -110, sdelay);
  StepX('D', 50, -5, 50, -80,-110, sdelay);
}

void Walk_Right(int cycles, int speeD){
  PreWalkRight(speeD);
  for(int i=0;i<cycles;i++){
    WalkRightOnce(speeD, speeD);
  }
  PostWalkRight(speeD);
}
/////////////////////////////////////////////



//LEFT WALK
void PreWalkLeft(int sdelay){
  StepX('C', -50, 50, -5, -80, -110, sdelay);
  StepX('B', -50, -50, 5, -80,-110, sdelay);
}

void WalkLeftOnce(int sdelay, int mdelay){
    StepX('C', -50, -5, 105, -80, -120, sdelay);
    
    MoveX(50, -50, 5, 105, 50, -55, -110,mdelay);

    StepX('A',50, -105, 5, -80, -110, sdelay);

    StepX('D', 50, -5, 105, -80, -110, sdelay);
    
    MoveX(50, 5, -50, 50, 105, -55, -110,mdelay);
  
    StepX('B', -50, -105, 5, -80, -110, sdelay);
}

void PostWalkLeft(int sdelay){
  StepX('C', -50, -5, 50, -80,-110, sdelay);
  StepX('B', -50, 5, -50, -80,-110, sdelay);
}

void Walk_Left(int cycles, int speeD){
  PreWalkLeft(speeD);
  for(int i=0;i<cycles;i++){
    WalkLeftOnce(speeD, speeD);
  }
  PostWalkLeft(speeD);
}


//BACKWARDS WALK
void PreWalkBackwards(int sdelay){
  StepY('D', 50, 50, -5, -80, -110, sdelay);
  StepY('C', 50, -50, 5, -80,-110, sdelay);
}

void WalkBackwardsOnce(int sdelay, int mdelay){
    StepY('D', 50, -5, 105, -80, -120, sdelay);

    MoveY(50, 50, -50, 5, 105, -55, -110,mdelay);
  
    StepY('B',-50, -105, 5, -80, -110, sdelay);

    StepY('A', -50, -5, 105, -80, -110, sdelay);
    
    MoveY(50, 105, 5, -50, 50, -55, -110,mdelay);
  
    StepY('C', 50, -105, 5, -80, -110, sdelay);  
}

void PostWalkBackwards(int sdelay){
  StepY('D', 50, -5, 50, -80,-110, sdelay);
  StepY('C', 50, 5, -50, -80,-110, sdelay);
}

void Set_Body_Rotation(float fan, float z_start, float x_angle, float y_angle){

  float z_delta_y = 45*sin(y_angle);
  float z_delta_x = 45*sin(x_angle);
  
  Set_Leg_Position_OCartes('A', -fan, fan, z_start + z_delta_y + z_delta_x) ;
  Set_Leg_Position_OCartes('B', -fan, -fan, z_start + z_delta_y - z_delta_x);
  Set_Leg_Position_OCartes('C', fan, -fan, z_start - z_delta_y - z_delta_x);
  Set_Leg_Position_OCartes('D', fan, fan, z_start - z_delta_y + z_delta_x);

}


void Walk_Backwards(int cycles, int speeD){
  PreWalkBackwards(speeD);
  for(int i=0;i<cycles;i++){
    WalkBackwardsOnce(speeD, speeD);
  }
  PostWalkBackwards(speeD);
}

///////////////////////////////////////////////////////////////////////////



//FORWARDS WALK
void PreWalkForwards(int sdelay, int stepheight/*-80*/){
  StepY('B', -50, -50, 5, stepheight, -110, sdelay);
  StepY('A', -50, 50, -5, stepheight,-110, sdelay);
}

void PostWalkForwards(int sdelay, int stepheight){
  StepY('B', -50, 5, -50, stepheight,-110, sdelay);
  StepY('A', -50, -5, 50, stepheight,-110, sdelay);
}

void WalkForwardsOnce(int sdelay, int mdelay, int stepheight){
  //  POSE1
  //  Set_Leg_Position_OCartes('A', -50,-5,-110);
  //  Set_Leg_Position_OCartes('B', -50,5,-110);
  //  Set_Leg_Position_OCartes('C', 50,-50,-110);
  //  Set_Leg_Position_OCartes('D', 50,50,-110);
    
  //STEP1B
    StepY('B', -50, 5, -105, stepheight, -120, sdelay);
      
  //POSE2
  //  Set_Leg_Position_OCartes('A', -50,-5,-110);
  //  Set_Leg_Position_OCartes('B', -50,-105,-120);
  //  Set_Leg_Position_OCartes('C', 50,-50,-110);
  //  Set_Leg_Position_OCartes('D', 50,50,-110);
    MoveY(50, -5, -105, -50, 50, 55, -110,mdelay);
  //POSE3
  //  Set_Leg_Position_OCartes('A', -50,50,-110);
  //  Set_Leg_Position_OCartes('B', -50,-50,-110);
  //  Set_Leg_Position_OCartes('C', 50,5,-110);
  //  Set_Leg_Position_OCartes('D', 50,105,-120);
  
  //STEP2D
    StepY('D',50, 105, -5, stepheight, -110, sdelay);
    
  
  //POSE4
  //  Set_Leg_Position_OCartes('A', -50,50,-110);
  //  Set_Leg_Position_OCartes('B', -50,-50,-110);
  //  Set_Leg_Position_OCartes('C', 50,5,-110);
  //  Set_Leg_Position_OCartes('D', 50,-5,-110);
  
    //STEP3C
    StepY('C', 50, 5, -105, stepheight, -110, sdelay);
    
  //  POSE5
  //  Set_Leg_Position_OCartes('A', -50,50,-110);
  //  Set_Leg_Position_OCartes('B', -50,-50,-110);
  //  Set_Leg_Position_OCartes('C', 50,-105,-120);
  //  Set_Leg_Position_OCartes('D', 50,-5,-110);
    MoveY(50, 50, -50, -105, -5, 55, -110,mdelay);
  //  POSE6
  //  Set_Leg_Position_OCartes('A', -50,105,-120);
  //  Set_Leg_Position_OCartes('B', -50,5,-110);
  //  Set_Leg_Position_OCartes('C', 50,-50,-110);
  //  Set_Leg_Position_OCartes('D', 50,50,-110);
  
    //STEP4A
    StepY('A', -50, 105, -5, stepheight, -110, sdelay);  
}

void Walk_Forwards(int cycles, int speeD, int stepheight){
  PreWalkForwards(speeD,stepheight);
  for(int i=0;i<cycles;i++){
    WalkForwardsOnce(speeD, speeD,stepheight);
  }
  PostWalkForwards(speeD,stepheight);
}
////////////////////////////////////////////////////////////////////////

void Rotate_Clockwise(int cycles, int speeD){
for(int it=0;it<cycles;it++){
  Serial.println(it);  
    for(int i=0;i<5;i++){
      Set_Leg_Position_OCartes('A', map(i,0,5,-50,-70), map(i,0,5,50,70),-110);
      Set_Leg_Position_OCartes('B', map(i,0,5,-50,-70), map(i,0,5,-50,-30),-110);
      Set_Leg_Position_OCartes('C', map(i,0,5,50,30), map(i,0,5,-50,-30),-110);
      Set_Leg_Position_OCartes('D', map(i,0,5,50,30), map(i,0,5,50,70),-110);
      delay(speeD);
    }
    StepXY('A', -70,-30,70,110,-75,-100,speeD);
    for(int i=0;i<5;i++){
      Set_Leg_Position_OCartes('A', map(i,0,5,-30,-10), map(i,0,5,110,90),-110);
      Set_Leg_Position_OCartes('B', map(i,0,5,-70,-50), map(i,0,5,-30,-50),-110);
      Set_Leg_Position_OCartes('C', map(i,0,5,30,50), map(i,0,5,-30,-50),-110);
      Set_Leg_Position_OCartes('D', map(i,0,5,30,50), map(i,0,5,70,50),-110);
      delay(speeD);
    }
    StepXY('D', 50, 90, 50, 10, -80, -110, speeD);
    StepXY('C', 50, 10, -50, -90, -80, -110,speeD);
    for(int i=0;i<5;i++){
      Set_Leg_Position_OCartes('A', map(i,0,5,-10,-30), map(i,0,5,90,70),-110);
      Set_Leg_Position_OCartes('B', map(i,0,5,-50,-70), map(i,0,5,-50,-70),-110);
      Set_Leg_Position_OCartes('C', map(i,0,5,10,-10), map(i,0,5,-90,-110),-110);
      Set_Leg_Position_OCartes('D', map(i,0,5,90,70), map(i,0,5,10,-10),-110);
      delay(speeD);
    }
    StepXY('B', -70, -110, -70, -30, -80, -110,speeD);
    
    for(int i=0;i<5;i++){
      Set_Leg_Position_OCartes('A', map(i,0,5,-30,-50), map(i,0,5,70,50),-110);
      Set_Leg_Position_OCartes('B', map(i,0,5,-110,-50), map(i,0,5,-30,-50),-110);
      Set_Leg_Position_OCartes('C', map(i,0,5,-10,50), map(i,0,5,-110,-50),-110);
      Set_Leg_Position_OCartes('D', map(i,0,5,70,50), map(i,0,5,-10,50),-110);
      delay(speeD);
    } 
  }
}

void Rotate_AntiClockwise(int cycles, int speeD){
for(int it=0;it<cycles;it++){
  Serial.println(it);  
    for(int i=0;i<5;i++){
      Set_Leg_Position_OCartes('A', map(i,0,5,-50,-70), map(i,0,5,50,70),-110);
      Set_Leg_Position_OCartes('B', map(i,0,5,-50,-70), map(i,0,5,-50,-30),-110);
      Set_Leg_Position_OCartes('C', map(i,0,5,50,30), map(i,0,5,-50,-30),-110);
      Set_Leg_Position_OCartes('D', map(i,0,5,50,30), map(i,0,5,50,70),-110);
      delay(speeD);
    }
    StepXY('A', -70,-110,70,30,-80,-100,speeD);
    for(int i=0;i<5;i++){
      Set_Leg_Position_OCartes('A', map(i,0,5,-110,-90), map(i,0,5,30,10),-110);
      Set_Leg_Position_OCartes('B', map(i,0,5,-70,-50), map(i,0,5,-30,-50),-110);
      Set_Leg_Position_OCartes('C', map(i,0,5,30,50), map(i,0,5,-30,-50),-110);
      Set_Leg_Position_OCartes('D', map(i,0,5,30,50), map(i,0,5,70,50),-110);
      delay(speeD);
    }
    StepXY('D', 50, 10, 50, 90, -80, -110, speeD);
    StepXY('C', 50, 90, -50, -10, -80, -110,speeD);
    for(int i=0;i<5;i++){
      Set_Leg_Position_OCartes('A', map(i,0,5,-90,-110), map(i,0,5,10,-10),-110);
      Set_Leg_Position_OCartes('B', map(i,0,5,-50,-70), map(i,0,5,-50,-70),-110);
      Set_Leg_Position_OCartes('C', map(i,0,5,90,70), map(i,0,5,-10,-30),-110);
      Set_Leg_Position_OCartes('D', map(i,0,5,10,-10), map(i,0,5,90,70),-110);
      delay(speeD);
    }
    StepXY('B', -70, -30, -70, -110, -80, -110,speeD);
    
    for(int i=0;i<5;i++){
      Set_Leg_Position_OCartes('A', map(i,0,5,-110,-50), map(i,0,5,-10,50),-110);
      Set_Leg_Position_OCartes('B', map(i,0,5,-30,-50), map(i,0,5,-110,-50),-110);
      Set_Leg_Position_OCartes('C', map(i,0,5,70,50), map(i,0,5,-30,-50),-110);
      Set_Leg_Position_OCartes('D', map(i,0,5,-10,50), map(i,0,5,70,50),-110);
      delay(speeD);
    } 
  }
}


