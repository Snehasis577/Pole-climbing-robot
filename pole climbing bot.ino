
// import necessary header files
#include <PS4USB.h>
#include <Servo.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif
// creat objects
USB Usb;
Servo myservo; 
PS4USB PS4(&Usb);
//int upd1 = 25;
//int upd2 = 34;
//int pld = 35;
//int plp = 5;
//int qrd = 33;
//int qrp = 4;
int spamp=30;
float scale_factor = 1;
//       motor configuration:
//---------------------------------------------
//     | motor2(fld,fla)   |   motor1(frd,fra) |
//---------------------------------------------
//     | motor3(bld,bla)   |   motor4(brd,bra) |
//---------------------------------------------
// motor pin variables
//byte frd = 47, fld = 49, bld = 33, brd = 43, fra = 11, fla = 12, bla = 4, bra = 9;// available pins
byte frd=31,fld=35,bld=37,brd=23,fra=8,fla=9,bla=11,bra=4;
int  d1, d2, d3, d4;
byte ser_pin=7,pole_motor=29,polem_dir=3;
//bla=8;
//bld=41;
// inverse kinematics variables
int w, x, y, xl, yl, ps = 90;
float a11 = -0.35, a12 = 0.35, a13 = 0.25;
float a21 = -0.35, a22 = -0.35, a23 = 0.25;
float a31 = 0.35, a32 = -0.35, a33 = 0.25;
float a41 = 0.35, a42 = 0.35, a43 = 0.25;

// speed variables.................
float sp = 3.0;
float s1, s2, s3, s4;
// flag variables
byte servo_lock=0,climb=0;
void setup() {
  //initialise pins for motor
  myservo.attach(ser_pin);
  pinMode(fra, OUTPUT);
  pinMode(frd, OUTPUT);
  pinMode(fla, OUTPUT);
  pinMode(fld, OUTPUT);
  pinMode(bla, OUTPUT);
  pinMode(bld, OUTPUT);
  pinMode(bra, OUTPUT);
  pinMode(brd, OUTPUT);
  pinMode(pole_motor, OUTPUT);
  pinMode(polem_dir, OUTPUT);
  Serial.begin(115200);// initialise serial communication
  // ps4 library setup
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 USB Library Started"));
}


void loop() {
  Usb.Task();
  if (PS4.connected())
  {

    if (PS4.getButtonClick(UP))// increase speed of motion
    {

      if (sp <= 10)
      { sp = sp + 0.5;

        Serial.print(F("\r\nUp\n"));
        Serial.print(sp);
        PS4.setLed(Red);
      }
    }
    if (PS4.getButtonClick(DOWN))// decrease speed of motion
    {

      if (sp > 0)
      { sp = sp - 0.5;
        Serial.print(F("\r\nDOWN\n"));
        Serial.print(sp);
        PS4.setLed(Green);
      }
    }
   
    if (PS4.getButtonClick(L1))
    {
     
     servo_lock=!servo_lock;
     if(servo_lock)
     {
      for (int pos = 0; pos <= 180; pos += 1) { // servo moves from 0 degrees to 180 degrees in steps of 1 degree
        myservo.write(pos);              
        delay(5);              
     }
     }
     else
     {
      analogWrite(pole_motor,0);//STOP POLE MOTORS FIRST 
      for (int pos = 180; pos >= 0; pos -= 1) { // Servo moves from 180 degrees to 0 degrees
         myservo.write(pos);              
          delay(5);                       
     }
     }
    }
    if (PS4.getButtonClick(R1))
    {
      climb=!climb;
      if(climb)
      {
        digitalWrite(polem_dir, LOW);//CLIMB UP
        analogWrite(pole_motor,180);
      }
      else
      {
        digitalWrite(polem_dir, HIGH);//CLIMB DOWN
        analogWrite(pole_motor,180);
      }
        
      }
     
    

    if (PS4.getAnalogHat(RightHatX) > 157 | PS4.getAnalogHat(RightHatX) < 100)
    {
      //get xy postion values of left and righ analog hats
      // x & y are used for heading in direction relative to head of robot
      x=PS4.getAnalogHat(RightHatX);
      y=PS4.getAnalogHat(RightHatY);
      xl = PS4.getAnalogHat(LeftHatX);
      yl = PS4.getAnalogHat(LeftHatY);
      
      // print the values
      Serial.print("x=");
      Serial.print(x);
      Serial.print(" y=");
      Serial.print(y);
      Serial.print(" s1=");
      Serial.print(s1);
      Serial.print(" s2=");
      Serial.print(s2);
      Serial.print(" s3=");
      Serial.print(s3);
      Serial.print(" s4=");
      Serial.println(s4);
      
      // inverse kinematics aLgorithm to compute speeds of motor for heading
      x -= 128;
      y -= 128;
      x = -x;
      if (xl < 57)
      {
        w = xl-127;    
      }
      else if (xl > 197)
      { w = xl-127;
      }
      else w = 0;
      s1 = sp * ((a11 * x) + (a12 * y) + (a13 * w));
      s2 = sp * ((a21 * x) + (a22 * y) + (a23 * w));
      s3 = sp * ((a31 * x) + (a32 * y) + (a33 * w));
      s4 = sp * ((a41 * x) + (a42 * y) + (a43 * w));
      if (s1 < 0)
      {
        s1 = -s1;
        d1 = 1;
      }
      else
      {
        d1 = 0;
      }
      if (s2 < 0)
      {
        s2 = -s2;
        d2 = 1;
      }
      else
      {
        d2 =0;
      }
      if (s3 < 0)
      {
        s3 = -s3;
        d3 = 0;
      }
      else
      {
        d3 = 1;
      }

      if (s4 < 0)
      {
        s4 = -s4;
        d4 = 1;
      }
      else
      {
        d4 = 0;
      }
      
      motion();
    }

    else if (PS4.getAnalogHat(LeftHatX)< 57  )//yaw left
    {
      digitalWrite(frd, 1);
      digitalWrite(fld, 1);
      digitalWrite(brd, 1);
      digitalWrite(bld, 0);
      analogWrite(fra, 70);
      analogWrite(fla, 70);
      analogWrite(bla, 70);
      analogWrite(bra, 70);
    }
    else if ( PS4.getAnalogHat(LeftHatX)> 197 )// yaw right
    {
      digitalWrite(frd, 0);
      digitalWrite(fld, 0);
      digitalWrite(brd, 0);
      digitalWrite(bld, 1);
      analogWrite(fra, 70);
      analogWrite(fla, 70);
      analogWrite(bla, 70);
      analogWrite(bra, 70);
    }

    


    else
    {
      stopp();

    }

}
}
void stopp()// stop function for base motors
{
  digitalWrite(frd, d1);
      digitalWrite(fld, d2);
      digitalWrite(brd, d4);
      digitalWrite(bld, d3);
      analogWrite(fra, 0);
      analogWrite(fla, 0);
      analogWrite(bla, 0);
      analogWrite(bra, 0);
}

void motion()// function to command base motors
{

  digitalWrite(frd, d1);
  digitalWrite(fld, d2);
  digitalWrite(brd, d4);
  digitalWrite(bld, d3);
  analogWrite(fra, s1);
  analogWrite(fla, s2);
  analogWrite(bla, s3);
  analogWrite(bra, s4);

}
