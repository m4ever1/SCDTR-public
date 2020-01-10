#include<Arduino.h>
#include"defs.h"

// int ledPin = 3;      // LED connected to digital pin 9
// int analogPin = A0;   // potentiometer connected to analog pin 3
float val = 0;// variable to store the read value
int dcycleper = 0; //default duty cycle of 10 out of 255
int dcycle = 0;

long currt;
// const byte mask= B11111000; // mask bits that are not prescale
int prescale = 1; //fastest possible
int dcycleinc = 5;
float R1 = 10000;
float vcc = 5.0;
float x = 50;

float vi;
float R2;
float tau;
float vf;
float dccref;
float dcc;
float simret;

float lbound;
float ubound;

float Kp = 0.0008;
float Ki = 50;
float Kd = 0;
float a = 0;
float y_ant = 0, i_ant = 0, d_ant = 0, e_ant = 0;
float T = 0.01;

float K1 = Kp;
float K2 = Kp*Ki*T/2;
float K3 = 0;//Kd/(Kd+a*T);
float K4 = 0;//Kp*Kd*a/(Kd+a*T);

bool occ = 1;

unsigned long ti;

volatile bool flag;

ISR(TIMER1_COMPA_vect){
  flag = 1; //notify main loop
}

void reasure(int recv)
{
  Serial.print("      Received: ");
  Serial.print(recv);
  Serial.println();
}

void wait4input()
{
   while(Serial.available() == 0);
}
  
float calcvf(float R2)
{
  float vf = vcc*((R1)/(R1 + R2));
  return vf;
}

float calcR2(float x)
{
  float m = -0.76;
  float b = 5.0337;
  if(x == 0)
  {
    x = 1;
  }
  float R2 = pow(10,((m*log10(x))+b));
  return R2;
}

float volttodc(float v)
{
  float a = 0.06325;
  float b = 0.001834;
  float dc;
  dc = a*exp(b*v);
  return dc;
}

float luxtodc(float lux)
{
  float R2 = calcR2(lux);
  float V = 1000*calcvf(R2);
  return volttodc(V);
}



float calctau(float R2)
{
  float c = 0.00001;
  float tau = 1000*((R1*R2)/(R1 + R2))*c;

  return tau;
}

float volttolux(float volt) //certa
{
  float a = 0.709;
  float b = 0.00132;
  return(a*exp(b*volt));
}

float simulation(float R2,float vf, float vi, float tau, unsigned long t)
{
  float  v = vf - (vf -vi)*exp(-(t/tau));
  return(v);
}

float DANGERZONE(float e)
{
   if(abs(e) < 50)
   {
    e = 0;
   }
   return e;
}

float pid(float ref, float y)
{
  float e = ref - y;
//  Serial.print(" e = ");
//  Serial.print(e);
  e = DANGERZONE(e);
  float p = K1*e;
  float i = i_ant + K2*(e + e_ant);
  if (i > 200)
  {
     i = 200;
  }
  if(i < -200)
  {
    i = -200;
  }
  float d = K3*d_ant - K4*(y - y_ant);
  float u = p+i+d;
  y_ant = y;
  i_ant = i;
  d_ant = d;
  e_ant = e;

  return u;
}

void set_bounds()
{
  Serial.println("Lower bound:");
  wait4input(); 
  lbound = Serial.parseInt();
 
  reasure(lbound);
  
  Serial.println("Upper bound:");
  wait4input();  
  ubound = Serial.parseInt();
  reasure(ubound);
}

void evaluate_x()
{
  if(occ == 1)
  {
    x = ubound;
  }
  else
  {
    x = lbound;
  }
}


int set_occ(int occ_t, int x)
{
  val = analogRead(analogPin);  // read the input pin
  val = map(val, 0, 1023, 0, 5000);
  occ = occ_t;
  
  vi = val/1000;
  R2 = calcR2(x);
  tau = calctau(R2);
  vf = calcvf(R2);
  dccref = luxtodc(x);
  
  return dccref;
}


// void setup() 
// {
//   Serial.begin(115200);
//   pinMode(ledPin, OUTPUT);  // sets the pin as output
//   Serial.println("start");
//   analogWrite(ledPin, 0);
//   x = 0;
//   TCCR2B = TCCR2B & B11111000 | B00000001;

 
  
//   // TIMER 1 for interrupt frequency 100 Hz:
//   cli(); // stop interrupts
//   TCCR1A = 0; // set entire TCCR1A register to 0
//   TCCR1B = 0; // same for TCCR1B
//   TCNT1  = 0; // initialize counter value to 0
//   // set compare match register for 100 Hz increments
//   OCR1A = 19999; // = 16000000 / (8 * 100) - 1 (must be <65536)
//   // turn on CTC mode
//   TCCR1B |= (1 << WGM12);
//   // Set CS12, CS11 and CS10 bits for 8 prescaler
//   TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
//   // enable timer compare interrupt
//   TIMSK1 |= (1 << OCIE1A);
//   sei(); // allow interrupts

//   startt = millis();

//   set_bounds(); 
//   set_occ();

//   Serial.print(" DCref = ");
//   Serial.print(dccref);   
  
// }



// void loop() 
// {
//   if(flag == 1)
//   {
// //    Serial.println("Input desired luxs");
// //    while(Serial.available() <= 0)
// //    {
// //    }
// //    x = Serial.parseFloat();
// //    Serial.print("LUX = ");
// //    Serial.println(x);


// //      float pidret = pid(vf, analogRead(analogPin));
// //      Serial.print(" Time ");
// //      Serial.print(millis()-ti);



//       analogWrite(ledPin,(int)map(dcc, 0, 100, 0,255));
//       val = analogRead(analogPin);
//       val = map(val, 0, 1023, 0, 5000);
//       simret = simulation(R2, vf, vi, tau, (millis()-ti));
//       Serial.print(" Voltage SIM ");
//       Serial.print(simret*1000);         
//       //float e = simret*1000 - val; 
      
//       dcc = pid(simret*1000, val) + dccref;
//       //Serial.print(" pidret = ");
//       //Serial.print(pid(simret*1000, val));
//       if(dcc > 100)
//       {
//         dcc = 100;
//       }
//       if (dcc < 0)
//       {
//         dcc = 0;
//       }
//       Serial.print(" DC = ");
//       Serial.print((int)dcc);
//       Serial.print(" Voltage MEAS ");      
//       Serial.print(val);
//       Serial.print(" i_ant = ");
//       Serial.println(i_ant);
//   }
//   flag = 0;
//   if(Serial.available() > 0)
//   {
//     wait4input();
//     set_occ();
//   }
  
// }
//    Serial.print("Vi ");
//    Serial.print(vi);
//    Serial.print(" x ");
//    Serial.print(x);
//    Serial.print(" R2 ");
//    Serial.print(R2);
//    Serial.print(" tau ");
//    Serial.print(tau);
//    Serial.print(" vf ");
//    Serial.print(vf);            
//    Serial.print(" ti ");
//    Serial.println(ti);
//  dcycle = map(dcycleper, 0, 100, 0, 255);
//  Serial.print("dcycle ");
//  Serial.println(dcycleper, DEC);
//    Serial.print("Time ");
//    Serial.print(millis());
//    Serial.print(" Voltage ");
//    Serial.print(val); 
