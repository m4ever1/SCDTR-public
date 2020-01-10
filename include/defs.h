#ifndef INDV_H
#define INDV_H
#include <stdio.h>
#include <Arduino.h>
#include "consensus.h"
#include <mcp2515.h>


#include "../include/lists.h"

union ID{

    unsigned long value;
    unsigned char bytes[4];
};
union my_can_msg{ //to pack/unpack long ints into bytes

    unsigned long value;
    unsigned char bytes[4];
};

extern uint32_t filt0; //accepts msg ID 1 on RXB0
extern uint32_t filt1; //accepts msg ID 2 on RXB1
extern uint32_t filt2; //accepts msg ID 3 on RXB1

extern uint32_t mask;
extern unsigned long SYNCTIME; //Time that the LED is on while synchronizing
extern unsigned long BD_TIME; //ID broadcasting time
extern int ledPin;      // LED connected to digital pin 3
extern int analogPin;   // potentiometer connected to analog pin 0
extern int* ID_ARRAY;
extern int ID_ARRAY_LENGHT;
extern float Ext_Lux;
extern float Ref_Lux;
extern float energy_spent;
extern float Vis_r;
extern float Flic_e;
extern int CONSENSUS_FLAG;
extern float LUX_ref;

extern volatile bool interrupt; //notification for ISR and loop()
extern volatile bool mcp2515_overflow;
extern volatile bool arduino_overflow;
extern unsigned long counter;
extern unsigned long test;

extern id_list local_id_list;

extern float val;// variable to store the read value
extern int dcycleper; //default duty cycle of 10 out of 255
extern int dcycle;
extern unsigned long startt;
extern long currt;
extern int prescale; //fastest possible
extern int dcycleinc;
extern float R1;
extern float vcc;
extern float x;

extern float vi;
extern float R2;
extern float tau;
extern float vf;
extern float dccref;
extern float dcc;
extern float simret;

extern float lbound;
extern float Lbound;

extern float Kp;
extern float Ki;
extern float Kd;
extern float a;
extern float y_ant, i_ant, d_ant, e_ant;
extern float T;

extern float K1;
extern float K2;
extern float K3;//Kd/(Kd+a*T);
extern float K4;//Kp*Kd*a/(Kd+a*T);

extern bool occ;
extern bool OCC_FLAG;

extern unsigned long ti;

extern volatile bool flag;

ISR(TIMER1_COMPA_vect);

MCP2515::ERROR write(uint32_t id, uint32_t val);
void reasure(int recv);
void wait4input();
float calcvf(float R2);
float calcR2(float x);
float volttodc(float v);
float luxtodc(float lux);
float calctau(float R2);
float volttolux(float volt);
float simulation(float R2,float vf, float vi, float tau, unsigned long t);
float DANGERZONE(float e);
float pid(float ref, float y);
void set_bounds();
void evaluate_x();
int set_occ(int occ_t, int x);


#endif