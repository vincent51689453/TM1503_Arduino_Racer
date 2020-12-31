#include "BD7411.h"
#include <SoftwareSerial.h>

/***************************
 * HC05 Bluetooth Module
 * HC05 TX -> 11 (Arduino MEGA)
 * HC05 RX -> 10 (Arduino MEGA)
***************************/
SoftwareSerial HC05(11,10); 


/***************************
 * GY-61 Accelerometer
***************************/
const int xPin = A3;
const int yPin = A2;
const int zPin = A1;

//The minimum and maximum values that came from
//the accelerometer while standing still
//You very well may need to change these
int minVal = 265;
int maxVal = 402;

//to hold the caculated values
double x;
double y;
double z;

//bluetooth message handler
char RX_Message = 'X';
bool start_flag = true;
bool initialize = false;
float new_gx;
float new_gy;
float new_gz;


//Interrupt Count
volatile bool H1_IN_OUT = false;
int H1_start = 0;
int H1_end = 0;
float H1_period = 0;
int H1_idle = 0;
int H1_old = 0;

volatile bool H2_IN_OUT = false;
int H2_start = 0;
int H2_end = 0;
float H2_period = 0;
int H2_idle = 0;
int H2_old = 0;

volatile bool H3_IN_OUT = false;
int H3_start = 0;
int H3_end = 0;
float H3_period = 0;
int H3_idle = 0;
int H3_old = 0;

volatile bool H4_IN_OUT = false;
int H4_start = 0;
int H4_end = 0;
float H4_period = 0;
int H4_idle = 0;
int H4_old = 0;

int hallout_pin = 0; // use D0 pin

BD7411 H1;
BD7411 H2;
BD7411 H3;
BD7411 H4;


bool H1_on_state=false;
bool H2_on_state=false;
bool H3_on_state=false;
bool H4_on_state=false;

float H1_hall_count=0;
float H2_hall_count=0;
float H3_hall_count=0;
float H4_hall_count=0;

int anchor = false;
int loop_count = 0;

float gx_buffer = 0.0;
float gy_buffer = 0.0;
float gz_buffer = 0.0;

float gx_offset = 0.0;
float gy_offset = 0.0;
float gz_offset = 0.0;

void setup()
{
  Serial.begin(115200);
  HC05.begin(115200); 
  
  H1.init(18);
  H2.init(19);
  H3.init(20);
  H4.init(21);
  
  //=============================================
  //External interrupt pin 18-21

  pinMode(18, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(18), counter_log_18, FALLING);

  pinMode(19, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(19), counter_log_19, FALLING);

  pinMode(20, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(20), counter_log_20, FALLING);

  pinMode(21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(21), counter_log_21, FALLING);

  pinMode(13, OUTPUT);
}

void counter_log_18()
{
  Serial.print("**********************");
  //Calculate the time between two falling edges
  if(H1_IN_OUT == false)
  {
    H1_start = 0;
    H1_end = 0;
    H1_period = 0;
    H1_start = millis();
    H1_IN_OUT = true;
    digitalWrite(13,LOW);
  }else{
    H1_end = millis();
    H1_period = abs(H1_end-H1_start);
    Serial.println(H1_period);
    digitalWrite(13,HIGH);
    H1_IN_OUT = false;
  }
  //H1_hall_count+=1.0;
}

void counter_log_19()
{
  //Calculate the time between two falling edges
  if(H2_IN_OUT == false)
  {
    H2_start = 0;
    H2_end = 0;
    H2_period = 0;
    H2_start = millis();
    H2_IN_OUT = true;
  }else{
    H2_end = millis();
    H2_period = abs(H2_end-H2_start);
    H2_IN_OUT = false;
  }
  //H2_hall_count+=1.0;
}

void counter_log_20()
{
  //Calculate the time between two falling edges
  if(H3_IN_OUT == false)
  {
    H3_start = 0;
    H3_end = 0;
    H3_period = 0;
    H3_start = millis();
    H3_IN_OUT = true;
  }else{
    H3_end = millis();
    H3_period = abs(H3_end-H3_start);
    H3_IN_OUT = false;
  }
  //H3_hall_count+=1.0;
}

void counter_log_21()
{
  //Calculate the time between two falling edges
  if(H4_IN_OUT == false)
  {
    H4_start = 0;
    H4_end = 0;
    H4_period = 0;
    H4_start = millis();
    H4_IN_OUT = true;
  }else{
    H4_end = millis();
    H4_period = abs(H4_end-H4_start);
    H4_IN_OUT = false;
  }
  //H4_hall_count+=1.0;
}
//=========================================

void bluetooth_control()
{
  while (HC05.available())
  {
     RX_Message = HC05.read(); 
     start_flag = true;
     if(RX_Message == 'A')
     {
       start_flag = true;
       anchor = false;
       Serial.print("Start\r\n");
     }else if(RX_Message == 'B')
            {
              start_flag = false;
              Serial.print("Stop\r\n");
            }
     
    RX_Message = 'X';
  }  
}

int curve_plotting(float a,float b,float c,float rpm_1,float rpm_2, float rpm_3, float rpm_4)
{
  int first;
  int second;
  int third;
  int r1;
  int r2;
  int r3;
  int r4;
  HC05.print("$");
  first = abs(a)*100;
  if(a<0)
  {
    HC05.print("-");
  }
  HC05.print(first);
  HC05.print(" ");
  
  second = abs(b)*100;
  if(b<0)
  {
    HC05.print("-");
  }
  HC05.print(second);
  HC05.print(" ");

  third = abs(c)*100;
  if(c<0)
  {
    HC05.print("-");
  }
  HC05.print(third);
  HC05.print(" "); 


  //plot rpm
  r1 = rpm_1;
  r2 = rpm_2;
  r3 = rpm_3;
  r4 = rpm_4;
  HC05.print(r1);
  HC05.print(" "); 
  HC05.print(r2);
  HC05.print(" ");
  HC05.print(r3);
  HC05.print(" ");
  HC05.print(r4);
  HC05.print(" ");
  HC05.print(";"); 
  return 0;
}

void loop()
{
  
  bluetooth_control();

  //gyro sensor work
  //read the analog values from the accelerometer
  int xRead = analogRead(xPin);
  int yRead = analogRead(yPin);
  int zRead = analogRead(zPin);

  // change the GYRO analog reading to g force

  float xvoltage = xRead * 5./1023;
  float Gx = (xvoltage - 2.5) / 0.36;
  float capture_Gx = 0.0;

  float yvoltage = yRead * 5./1023;
  float Gy = (yvoltage - 2.5) / 0.36;
  float capture_Gy = 0.0;

  float zvoltage = zRead * 5./1023;
  float Gz = (zvoltage - 2.5) / 0.36;
  float capture_Gz = 0.0;

  if(initialize == false)
  {
    gx_buffer += Gx;
    gy_buffer += Gy;
    gz_buffer += Gz ;
    if(loop_count == 1000)
    {
      //Take 300 samples to caculat the offset
      gx_offset = gx_buffer/loop_count;
      gy_offset = gy_buffer/loop_count;
      gz_offset = gz_buffer/loop_count;
      initialize = true;
    } 
    loop_count++;
    Serial.print("Calibrating accelerometer...\r\n"); 
  }else{
    capture_Gx = Gx - gx_offset;
    capture_Gy = Gy - gy_offset;
    capture_Gz = Gz - gz_offset;    
  }

  //Reset H1 period if no more interrupts were found
  if(H1_period == H1_old)
  {
    H1_idle += 1;
  }
  if(H1_idle >= 500)
  {
    H1_period = 0;
    H1_idle = 0;
  }
  //Find H1 rpm
  float wheel_H1 = 0.00;
  if(H1_period == 0.0)
  {
     wheel_H1 = 0.0; 
  }else{
     wheel_H1 = 1.0/((H1_period/1000)/60.0); //TO-DO: Define pulse for a rotation
  }
  
  //Reset H2 period if no more interrupts were found
  if(H2_period == H2_old)
  {
    H2_idle += 1;
  }
  if(H2_idle >= 500)
  {
    H2_period = 0;
    H2_idle = 0;
  }
  //Find H2 rpm
  float wheel_H2 = 0.00;
  if(H2_period == 0.0)
  {
     wheel_H2 = 0.0; 
  }else{
     wheel_H2 = 1/((H2_period/1000)/60); //TO-DO: Define pulse for a rotation
  }

  //Reset H3 period if no more interrupts were found
  if(H3_period == H3_old)
  {
    H3_idle += 1;
  }
  if(H3_idle >= 500)
  {
    H3_period = 0;
    H3_idle = 0;
  }
  //Find H3 rpm
  float wheel_H3 = 0.00;
  if(H3_period == 0.0)
  {
     wheel_H3 = 0.0; 
  }else{
     wheel_H3 = 1/((H3_period/1000)/60); //TO-DO: Define pulse for a rotation
  }

  //Reset H4 period if no more interrupts were found
  if(H4_period == H4_old)
  {
    H4_idle += 1;
  }
  if(H4_idle >= 500)
  {
    H4_period = 0;
    H4_idle = 0;
  }
  //Find H4 rpm
  float wheel_H4 = 0.00;
  if(H4_period == 0.0)
  {
     wheel_H4 = 0.0; 
  }else{
     wheel_H4 = 1/((H4_period/1000)/60); //TO-DO: Define pulse for a rotation
  }

  Serial.print("Wheel H1(Pulse per minute): ");
  Serial.print(wheel_H1);
  Serial.print("  Wheel H2(Pulse per minute): ");
  Serial.print(wheel_H2);
  Serial.print("  Wheel H3(Pulse per minute): ");
  Serial.print(wheel_H3);
  Serial.print("  Wheel H4(Pulse per minute): ");
  Serial.println(wheel_H4);

  curve_plotting(new_gx,new_gy,new_gz,wheel_H1,wheel_H2,wheel_H3,wheel_H4);
  
  H1_old = H1_period;
  H2_old = H2_period;
  H3_old = H3_period;
  H4_old = H4_period;

}
