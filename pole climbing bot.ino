#include <Servo.h>
bool f = 0;
int th=40;
Servo myservo;
//ldr pins
byte ldr = 25;
//nex line sensor  pins....................................
byte nex[7] = {A6, A5, A4, A3, A2, A1, A0};

//ultrasonic pins..........................................
byte aus = A13, dus = 53;

//servo pins...............................................
byte sersw = 23, sersig = 13;

//led pins.................................................
byte led[3] = {47, 49, 51};

//motordriver pins.........................................
byte ra = 2, rd = 3, la = 4, ld = 5, pr1 = 7, pr2 = 6;


//data variables.........................
float sv[7], ps[7];
int nexmin = 100;
int nexmax = 100;
byte ser_max=180,ser_min=90;
byte ud = 0, jwidth = 4;
int  ua = 0;

//SPEED VARIABLE..............
float speed_max = 220; //MAX SPEED OF MOTTORS
float speed_right = 0; //RIGHT MOTOR SPEED
float speed_left = 0; //LEFT MOTOR SPEED
byte turn_speed = 100;
bool dir_right = 0;
bool dir_left = 0;
byte pos;
byte dir = 0;


// delay variables
int fd = 50, td = 350, sd = 100;

// PID VARIABLE...............
float kp = 5.4; // CONSTANT KP 6
float ki = 0.025; // CONSTANT KI0.031  0.081
float kd = 50; // CONSTANT KD 50
double pid_val;//TOTAL pid_val=kp*error+ki*integral+kd*prop
float error, value, prop, last_error, integral, sum = 0, son = 0;
float limit = 2000;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);// initialise serial communication
  myservo.attach(13); // inititalise the servo
  myservo.write(180);// set servo position to 180 degrees

  // initalise the pins for sensor and motor asper requirement
  pinMode(ldr, INPUT);
  pinMode(aus, INPUT);
  pinMode(dus, INPUT);
  pinMode(sersw, INPUT);
  for (int i = 0; i < 7; i++)
    pinMode(nex[i], INPUT);
  pinMode(led[0], OUTPUT);
  pinMode(led[1], OUTPUT);
  pinMode(led[2], OUTPUT);
  pinMode(ra, OUTPUT);
  pinMode(rd, OUTPUT);
  pinMode(la, OUTPUT);
  pinMode(ld, OUTPUT);
  pinMode(pr1, OUTPUT);
  pinMode(pr2, OUTPUT);
  caliberate();
}
void pid_calculate()//function to calculate pid values
{
  // pid algorithm............................
  error = value - 36;               //36 is the requied position(set point)
  prop = error - last_error;
  integral = integral + error;
  integral = integral > limit ? limit : integral;
  integral = (integral < -limit && integral < 0) ? -limit : integral;
  last_error = error;
  pid_val = kp * error + ki * integral + kd * prop;
  //print the values
  Serial.println();
  Serial.print(" sum=   ");
  Serial.println(value);

  Serial.print("error  "); Serial.print(error);
  Serial.print("last_error  "); Serial.print(last_error);
  Serial.print("prop  "); Serial.print(prop);
  Serial.print("integral  "); Serial.print(integral);
  Serial.print("pid_val  "); Serial.println(pid_val);

}
void leftturn(int x) //function for turn left
{
  
  while (sv[0] == 0) {
    read_nex();
    analogWrite(ra, turn_speed);
    digitalWrite(rd, 0);
    analogWrite(la, turn_speed);
    digitalWrite(ld, 1);
    delay(x);
  }
}
void rightturn(int x) //function for turn left
{
  while (sv[6] == 0) {
    read_nex();
    analogWrite(ra, turn_speed);
    digitalWrite(rd, 1);
    analogWrite(la, turn_speed);
    digitalWrite(ld, 0);
    delay(x);
  }
}
void back(int x) //function for reverse direction motion
{
  analogWrite(ra, turn_speed);
  digitalWrite(rd, 1);
  analogWrite(la, turn_speed);
  digitalWrite(ld, 0);
  delay(x);
}
void forward(int x) // function for straight heading
{
  analogWrite(ra, speed_max - 100);
  digitalWrite(rd, 0);
  analogWrite(la, speed_max - 100);
  digitalWrite(ld, 0);
  delay(fd);
}
void motion()//function to command motors
{

  if (pid_val < 0)
  {
    speed_left = speed_max + pid_val;
    speed_right = speed_max;
    if (speed_left < 0)
      speed_left = 5;
  }
  else
  {
    speed_left = speed_max;
    speed_right = speed_max - pid_val;
    if (speed_right < 0)
      speed_right = 5;
  }


  analogWrite(ra, speed_right);
  analogWrite(la, speed_left);
  digitalWrite(rd, dir_right);
  digitalWrite(ld, dir_left);
}
void stopp()// sudden stop function
{
  analogWrite(ra, 70);
  analogWrite(la, 70);
  digitalWrite(rd, 1);
  digitalWrite(ld, 1);
  delay(sd);
  analogWrite(ra, 0);
  analogWrite(la, 0);
  digitalWrite(rd, 1);
  digitalWrite(ld, 1);
}
void stoppe()// function for free stop
{

  analogWrite(ra, 0);
  analogWrite(la, 0);
  digitalWrite(rd, 1);
  digitalWrite(ld, 1);
}

void read_nex()// function to read the nex line sensor
{

  son = 0;
  sum = 0;
  for (int j = 0; j < 7; j++)
  {
  //  sv[j] = !digitalRead(nex[j]) * (1023 - analogRead(nex[j]));   // for black background
      sv[j] = digitalRead(nex[j]) * (analogRead(nex[j]));   //for white background
      Serial.print(sv[j]);
      Serial.print("    ");
      sv[j] = map(sv[j],nexmin, nexmax, 0, 10);// map sensor value to 0-10
      Serial.print(sv[j]);
      // calculate weighted sum and  average value
    if (sv[j] >= 1)
    {
      son++;
      sum = sum + (sv[j] * (j + 1));

    }

    Serial.print("    ");
  }

  value = sum / son;
}
void print_nex()// function to print sensor value to serial monitor and store the current sensor value in ps[]
{
   for (int j = 0; j < 7; j++)
  {
    ps[j] = sv[j];
    Serial.print(sv[j]);
    Serial.print("    ");
  }
  Serial.println();
}

void ultra_read()// function to read ultrasonic sensor
{
  
  ud = digitalRead(dus);
  ua = analogRead(aus);
}
void caliberate()// function for automatic caliberation of line sensor
{
  digitalWrite(led[0], 1);
  nexmin = analogRead(nex[3]);
  nexmax = nexmin;
  for (int n = 0; n <= 5000; n++)
  {
    for (int j = 0; j < 7; j++)
    {
      sv[j] = analogRead(nex[j]);
      if (sv[j] > nexmax)
        nexmax = sv[j];
      if (sv[j] < nexmin)
        nexmin = sv[j];
    }
  }
  Serial.println(nexmax);
   Serial.println(nexmin);
  digitalWrite(led[0], 0);


}
//######################### LOOP FUNCTION #############################################//////////////////////////////////////////
void loop() {

  // put your main code here, to run repeatedly:
  read_nex();
  ultra_read();    //  pid_calculate();
  motion();
  if(ua<=33)
  door();
  // if all sensors have 0 value
  if (son == 0 )
  {
    digitalWrite(led[2], HIGH);
    stoppe();
    Serial.println(dir);
    digitalWrite(led[2], LOW);
    // go back if bot gets out of line go back to find the line
    while (sv[6] == 0 && sv[5] == 0 && sv[4] == 0 &&sv[3] == 0 && sv[2] == 0 && sv[1] == 0 && sv[0] == 0 )
    { read_nex();
      back(1);
    }

  }
  else if (son >= jwidth)// if junction detected
  {
    read_nex();
    //  read_nex();
    print_nex();

    dir = 1;
    stopp();
    stoppe();

    if (son >= jwidth + 2)
    {
      dir = 2;
    }

    read_nex();

    // read_nex();
    if (son >= 1)
    { dir = 3;
    }
    if (dir == 1)
    {
      if ((ps[0] >= 1 && ps[1] >= 1) && ps[2] >= 1 && ps[3] >= 1)
      {
        digitalWrite(led[1], HIGH);
        // forward(fd);
        //forward(20);

        leftturn(1);
        //  forward(20);
        digitalWrite(led[1], LOW);

      }
      else if (ps[3] >= 1 && ps[4] >= 1 && (ps[5] >= 1 || ps[6] >= 1))
      {
        digitalWrite(led[1], HIGH);
        //forward(fd);
        //  forward(20);

        rightturn(1);

        digitalWrite(led[1], LOW);
      }
    }
    else if (dir == 2)
    {
      // forward(20);

      leftturn(1);
      //rightturn(td);
    }
    else if (dir == 3)
    {
      digitalWrite(led[0], 1);
      digitalWrite(led[1], 1);
      if ((ps[0] >= 1 && ps[1] >= 1) && ps[2] >= 1 && ps[3] >= 1)
      {
        // forward(20);
        // forward(fd);

        leftturn(1);
      }
      else
      {
        pid_calculate();
        motion();
      }
      digitalWrite(led[0], 0);
      digitalWrite(led[1], 0);

    }
    //        stoppe();
    //        while (1)
    //        {
    //          print_nex();
    //          // stoppe();
    //        }
    //
    //      }
  }
  else
  {
    pid_calculate();
    motion();
  }
  //Serial.println("");
  //for (int j=0;j<7;j++)
  //  {sv[j]=analogRead(nex[j]);
  //   value = map(sv[j], 0, 255, 0, 10);
  //Serial.print(value);
  //Serial.print("    ");}
  //Serial.println("");


}
//###############################END OF LOOP #########################################
void start()
{
  f = digitalRead(23);

  for (pos = ser_max; pos >= ser_min; pos--)
    {
      myservo.write(pos);
      delay(15);
    }
  while (f)

  { Serial.write("caliberating sensors");
    f = digitalRead(23);
    Serial.println(analogRead(aus));


    if (analogRead(aus) <=th)
    {Serial.println(analogRead(aus));
      digitalWrite(led[0], HIGH);
    }
    else
      digitalWrite(led[0], 0);
      
    if (digitalRead(ldr) == 1)
      digitalWrite(led[1], HIGH);
    else
      digitalWrite(led[1], 0);
  }
  for (pos = ser_min; pos <= ser_max; pos++)
  {
    myservo.write(pos);
    delay(15);
  }

  
  
}
void door()
{
  stopp();
  for (pos =ser_max; pos >= ser_min; pos--)
    {
      myservo.write(pos);
      delay(15);
    }
   while(1)
   Serial.println("door found");
//  if(digitalRead(ldr) == 1)
//  {
//    while(digitalRead(ldr) == 1)
//    digitalWrite(27,HIGH);
//  }
//digitalWrite(27,LOW);
// for (pos = 110; pos <= 180; pos++)
//    {
//      myservo.write(pos);
//      delay(15);
//    }
  
}




