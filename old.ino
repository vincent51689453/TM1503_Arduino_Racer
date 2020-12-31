#include <BD7411.h>
#include <SoftwareSerial.h>

SoftwareSerial HC05(11,10); 
//HC05 TX -> 11
//HC05 RX -> 10

//Analog read pins
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

  
int hallout_pin = 0; // use D0 pin

BD7411 H1;
BD7411 H2;
BD7411 H3;
BD7411 H4;

//int H1_count=0;
//int H2_count=0;
//int H3_count=0;
//int H4_count=0;

int count = 0;
float t;
float start;
bool H1_on_state=false;
bool H2_on_state=false;
bool H3_on_state=false;
bool H4_on_state=false;

float H1_hall_count=0;
float H2_hall_count=0;
float H3_hall_count=0;
float H4_hall_count=0;

int anchor = false;

void setup()
{
  Serial.begin(115200);
  HC05.begin(115200); 
  HC05.print("I love jason");
  
  H1.init(18);
  H2.init(19);
  H3.init(20);
  H4.init(21);
  
  start = millis();
  t=start;

//=============================================
//interrupt pin 18-21

  pinMode(18, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(18), abc, FALLING);

  pinMode(19, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(19), abc, FALLING);

    pinMode(20, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(20), abc, FALLING);

    pinMode(21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(21), abc, FALLING);
}

void abc()
{
  if(!digitalRead(18))
    H1_hall_count+=1.0;
  
  if(!digitalRead(19))
    H2_hall_count+=1.0;

  if(!digitalRead(20))
    H3_hall_count+=1.0;

  if(!digitalRead(21))
    H4_hall_count+=1.0;
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
   capture_Gz = Gz;
   capture_Gy = Gy;
   capture_Gx = Gx;  
   initialize = true;
}


bluetooth_control();

////convert read values to degrees -90 to 90 – Needed for atan2
//int xAng = map(xRead, minVal, maxVal, -90, 90);
//int yAng = map(yRead, minVal, maxVal, -90, 90);
//int zAng = map(zRead, minVal, maxVal, -90, 90);
//
//VoltsRx = 586 * 3.3V / 1023; 
//VoltsRy = 630 * 3.3V / 1023;
//VoltsRz = 561 * 3.3V / 1023;
//
////Caculate 360deg values like so: atan2(-yAng, -zAng)
////atan2 outputs the value of -π to π (radians)
////We are then converting the radians to degrees
//x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
//y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
//z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

//Output the caculations
//Serial.print("x: ");
//Serial.print(x);
//Serial.print(" | y: ");
//Serial.print(y);
//Serial.print(" | z: ");
//Serial.println(z);

//delay(100);//just here to slow down the serial output – Easier to read


//hall sensor counting REV1
  
// set number of hall trips for RPM reading (higher improves accuracy)float hall_thresh = 100.0;
  
  // preallocate values for tach
  
  // counting number of times the hall sensor is tripped
  // but without double counting during the same trip
 
//Serial.println(H1_hall_count);
  float now = millis();
  if( (now - t) > 100)
  {
    if(start_flag == true)
    {
      /**
      HC05.print ("G: ");
      HC05.print(Gx);
      HC05.print (" ");
      HC05.print(Gy);
      HC05.print (" ");
      HC05.println(Gz);
      Serial.print ("G: ");
      Serial.print(Gx);
      Serial.print (" ");
      Serial.print(Gy);
      Serial.print (" ");
      Serial.println(Gz); 
      **/

      new_gx = Gx - capture_Gx + 2.6;
      new_gy = Gy - capture_Gy + 2.6;
      new_gz = Gz - capture_Gz + 3.5;
      //curve_plotting(new_gx,new_gy,new_gz);
      //curve_plotting(-100,100,-13);
          
    }else{
           if(anchor==false)
           {
             capture_Gz = Gz;
             capture_Gy = Gy;
             capture_Gx = Gx;
             /**
             Serial.print("Capture Gx:");
             Serial.print(capture_Gx);
             Serial.print(" Capture Gy:");
             Serial.print(capture_Gy);
             Serial.print(" Capture Gz:");
             Serial.print(capture_Gz);
             Serial.print("\r\n");
             Serial.print("Cal Gx:");
             Serial.print(Gx - capture_Gx);
             **/
             anchor = true;
           }
         }
    t = now;
    count++;

    if((count % 1 == 0)&&(start_flag == true))
    {
      float end_time = now;
      float time_passed = ((end_time-start)/1000.0);
      HC05.print("Time Passed: ");
      HC05.print(time_passed);
      HC05.println("s");
      Serial.print("Time Passed: ");
      Serial.print(time_passed);
      Serial.println("s");
      
      float rpm_val = (H1_hall_count/time_passed)*60000.0;
      float rp_1 = rpm_val;
      HC05.print(rpm_val);
      HC05.print(" H1_RPM  ");
      Serial.print(rpm_val);
      Serial.print(" H1_RPM  ");
  
      rpm_val = (H2_hall_count/time_passed)*60000.0;
      float rp_2 = rpm_val;
      HC05.print(rpm_val);
      HC05.print(" H2_RPM  ");
      Serial.print(rpm_val);
      Serial.print(" H2_RPM  ");
      
      rpm_val = (H3_hall_count/time_passed)*60.0;
      float rp_3 = rpm_val;
      HC05.print(rpm_val);
      HC05.print(" H3_RPM  ");
      Serial.print(rpm_val);
      Serial.print(" H3_RPM  ");  
      
      rpm_val = (H4_hall_count/time_passed)*60000.0;
      float rp_4 = rpm_val;
      HC05.print(rpm_val);
      HC05.println(" H4_RPM");
      Serial.print(rpm_val);
      Serial.println(" H4_RPM");
  
      H1_hall_count=0;
      H2_hall_count=0;
      H3_hall_count=0;
      H4_hall_count=0;

      curve_plotting(new_gx,new_gy,new_gz,rp_1,rp_2,rp_3,rp_4);
      start = now;
    }
  }
}
