#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include "lists.h"
#include "freemem.h"
#include "defs.h"
#include "case_fun.h"

unsigned long SYNCTIME = 4000; //Time that the LED is on while synchronizing
unsigned long BD_TIME = 3000; //ID broadcasting time
int ledPin = 3;      // LED connected to digital pin 3
int analogPin = A0;   // potentiometer connected to analog pin 0
int* ID_ARRAY = nullptr;
int ID_ARRAY_LENGHT = 0;
int OUT_DIST = 0;
int own_o = 0;
float LUX_ref;
float Ext_Lux;
float Ref_Lux;
float energy_spent = 0.0;
float Vis_r, Flic_e; 
float Lbound;
int CONSENSUS_FLAG = 1;
bool OCC_FLAG = 0;

unsigned long startt = millis();

MCP2515 mcp2515(10); //SS pin 10

volatile bool interrupt = false; //notification for ISR and loop()
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;
unsigned long counter = 0;
unsigned long test = 9;
float* Karray;
float* K_max_lux;

id_list local_id_list;
int own_id = EEPROM.read(0);
int last_id = 0;
int ID_IN_ARRAY = own_id;

float t_ant = 0.0;  
float  fi =0, Vis_r_all=0;
int N=1;


class can_frame_stream{

    static constexpr int buffsize = 10; //space for 10 can_msgs - increase if needed
    can_frame cf_buffer[buffsize];
    int read_index; //where to read the next msg
    int write_index; //where to write the next msg
    bool write_lock; //buffer full

    public:

    can_frame_stream()
    {
      read_index = 0;
      write_index = 0;
      write_lock = false;
    }//contructor/initial_state

    int put(can_frame &frame){
        if (write_lock) return 0; // buffer full
        cf_buffer[write_index] = frame;
        write_index = (++write_index)%buffsize;
        if (write_index == read_index) write_lock = true; // cannot write more
        return 1;
    }

    int get(can_frame &frame){

        if (!write_lock && (read_index==write_index)) return 0; //empty buffer
        if (write_lock && (read_index==write_index)) write_lock = false; //realese lock
        frame = cf_buffer[read_index];
        read_index = (++read_index)%buffsize;
        return 1;
    }
}volatile cf_stream; // the object to use


void irqHandler(){

    can_frame frame;
    uint8_t irq = mcp2515.getInterrupts(); //read CANINTF
    if (irq & MCP2515::CANINTF_RX0IF){ //msg in receive buffer 0
        mcp2515.readMessage(MCP2515::RXB0, & frame); //also clears RX0IF
        if(!cf_stream.put(frame)) arduino_overflow = true;
    }

    if (irq & MCP2515::CANINTF_RX1IF){ //msg in receive buffer 1
        mcp2515.readMessage(MCP2515::RXB1, & frame); //also clears RX1IF
        if(!cf_stream.put(frame)) arduino_overflow = true;
    }

    irq = mcp2515.getErrorFlags(); //read EFLG
    if((irq & MCP2515::EFLG_RX0OVR) | (irq & MCP2515::EFLG_RX1OVR)){
        mcp2515_overflow = true;
        mcp2515.clearRXnOVRFlags();
    }

    mcp2515.clearInterrupts();
    interrupt = true; // notify loop()
}



union my_can_signed{ //to pack/unpack long ints into bytes

    signed long value;
    unsigned char bytes[4];
};

MCP2515::ERROR write(uint32_t id, uint32_t val){

    can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 4;
    my_can_msg msg;
    msg.value = val;

    for(int i=0; i<4 ;i++) frame.data[i] = msg.bytes[i];
    return mcp2515.sendMessage(&frame);
}

MCP2515::ERROR read(unsigned long &c){

    can_frame frame;
    my_can_msg msg;
    MCP2515::ERROR err = mcp2515.readMessage(&frame);

    if (err == MCP2515::ERROR_OK){
        for(int i =0; i<4; i++) msg.bytes[i]=frame.data[i];
        c = msg.value;
    }
    return err;
}


can_frame get_frame()
{
    can_frame frame;
    interrupt = false;
    if(mcp2515_overflow){
        Serial.println("\t\tError: MCP2515 RX Buffers Full");
        mcp2515_overflow = false;
    }

    if(arduino_overflow){
        Serial.println("\t\tError: Arduino Stream Buffers Overflow");
        mcp2515_overflow = false;
    }

    while(cf_stream.get(frame));

    return frame;

}

void clean_buff()
{
    can_frame frame;
    for(int i = 0; i < 10; i++)
    {
       frame = get_frame();
    }
}

bool build_ID_list()
{
  can_frame frame;
  my_can_msg msg;
  ID write_id, recv_id;
  unsigned long startt = millis();

  write_id.bytes[0] = 0;
  write_id.bytes[1] = own_id;
  // delay(10*own_id);

  int count = 0;

  if( write(write_id.value, own_id) != MCP2515::ERROR_OK)
  {
    Serial.println("\t\tError: MCP2515 TX Buffers Full");
  }

  delay(1000);
  
  while(count < 50)
  {
    if(interrupt)
    {
      interrupt = false;
      if (mcp2515_overflow) {
        Serial.println("\t\tError: MCP2515 RX Buffers Full");
        mcp2515_overflow = false;
      }

      if (arduino_overflow) {
        Serial.println("\t\tError: Arduino Stream Buffers Overflow");
        arduino_overflow = false;
      }
      while(cf_stream.get(frame))
      {
        for(int i=0; i<4; i++) msg.bytes[i]=frame.data[i];


        if(msg.value != 0 && (local_id_list.is_in_list(msg.value) == 0))
        {
          Serial.print("List:");
          Serial.println(msg.value);
          local_id_list.addid(msg.value);
        }
      }
    }
    count++;
    delay(100);

  }
  delay(100);
}


float* init_sync()
{
  can_frame frame;
  my_can_msg msg;
  ID write_id, recv_id;
  auto* Karr = new float[ID_ARRAY_LENGHT]; // DONT FORGET TO DEALLOCATE (IF NEEDED)
  
  int nextid = 100;
  int timeout, curr_time=0;

  write_id.bytes[0] = 0;
  write_id.bytes[1] = own_id;
  msg.value = 1;


  if(own_id == ID_ARRAY[0])
  {
    ID_IN_ARRAY = 0;
    curr_time = millis();
    analogWrite(ledPin, 255);
    delay(SYNCTIME/2);
    Karr[0] = map(analogRead(analogPin), 0, 1023, 0, 5000);
    msg.value = 1;
    write(write_id.value, msg.value); //SENDS NEXT INDEX IN THE ID ARRAY
    delay(SYNCTIME/2);
    msg.value = 0;

    if( write(write_id.value, msg.value) != MCP2515::ERROR_OK)
    {
      Serial.println("\t\tError: MCP2515 TX Buffers Full");
    }

    delay(10);
    analogWrite(ledPin, 0);
  }
  while(true)
  {
    if(interrupt)
    {
      frame = get_frame();
      for(int i=0; i<4; i++) msg.bytes[i]=frame.data[i];
      Serial.print("\t GOT :");
      Serial.println(msg.value);

      if(msg.value != 0)
      {
        nextid = msg.value;
        Karr[msg.value - 1] = map(analogRead(analogPin), 0, 1023, 0, 5000);
        Serial.print("Saving in index:\t");
        Serial.println(msg.value -1);
      }
      else if(msg.value == 0)
      {
        if(ID_ARRAY[nextid] == own_id)
        {
          ID_IN_ARRAY = nextid;
          analogWrite(ledPin, 255);
          delay(SYNCTIME/2);
          Karr[nextid] = map(analogRead(analogPin), 0, 1023, 0, 5000);
          msg.value = nextid + 1;
          Serial.print("Saving in index:\t");
          Serial.println(nextid);
          write(write_id.value, msg.value); //SENDS NEXT INDEX IN THE ID ARRAY
          
          if( write(write_id.value, msg.value) != MCP2515::ERROR_OK)
          {
            Serial.println("\t\tError: MCP2515 TX Buffers Full");
          }
         
          delay(SYNCTIME/2);
          msg.value = 0;
          write(write_id.value, msg.value);
          delay(10);
          analogWrite(ledPin, 0);
          if(nextid+1 == ID_ARRAY_LENGHT)
          {
            break;
          }
        }

        else if(nextid == ID_ARRAY_LENGHT)
        {
          break;
        }

      }
    }
  }
  return(Karr);
}

void pseudo_setup()
{
  startt = millis();
  delete[] K_max_lux;
  auto* K_max_lux = new float[ID_ARRAY_LENGHT]; // DONT FORGET TO DEALLOCATE (IF NEEDED)
  Serial.println("PSEUDO SETUP");
  analogWrite(ledPin, 0);
  delay(2000);
  int a =0;
  
  ID write_id, recv_id;
  my_can_signed msg;
  can_frame frame;
  
  //    Serial.println("ID LIST IS");
  delay(100);
  delete[] Karray;
  Karray = init_sync();//each one tries to communicate with each other
  delay(1000);

  OUT_DIST = analogRead(analogPin);
  Serial.println("K ARRAY IS:");
  
  for(int i = 0; i< ID_ARRAY_LENGHT; i++) 
  {
    Karray[i] = volttolux(Karray[i]);
    Serial.println(Karray[i]);
  }

  K_max_lux[ID_IN_ARRAY] = Karray[ID_IN_ARRAY];
  write_id.bytes[0] = 0;
  write_id.bytes[1] = ID_IN_ARRAY;

  if( write(write_id.value, K_max_lux[ID_IN_ARRAY]) != MCP2515::ERROR_OK)
  {
    Serial.println("\t\tError: MCP2515 TX Buffers Full");
  }

  Serial.println("\t\tSent K\t");

  delay(500);

  while(a<ID_ARRAY_LENGHT-1)
  {
    if(interrupt)
    {
      interrupt = false;
      if (mcp2515_overflow) {
        Serial.println("\t\tError: MCP2515 RX Buffers Full");
        mcp2515_overflow = false;
      }

      if (arduino_overflow) {
        Serial.println("\t\tError: Arduino Stream Buffers Overflow");
        arduino_overflow = false;
      }
      while (cf_stream.get(frame)) 
      {
        for(int i=0; i<4; i++) msg.bytes[i]=frame.data[i];
        recv_id.value = frame.can_id;
        Serial.print("Saving:\t");
        Serial.println(msg.value);
        Serial.print("in:\t");
        Serial.println(recv_id.bytes[1]);

        K_max_lux[recv_id.bytes[1]] = msg.value; 
        a++;
      }

    }

    delay(100);
  }
  
  Serial.println("K_max_lux:");
  for(int i = 0; i< ID_ARRAY_LENGHT; i++) 
  {
    Serial.println(K_max_lux[i]);
  }

  own_o = map(OUT_DIST, 0, 1023, 0, 5000);
  own_o = volttolux(own_o);
  Ext_Lux = own_o;

}

void consensus_algorithm(c_node own_node)
{
    int a  = 0;
    int timeout = 0;
    int prev_dav[ID_ARRAY_LENGHT];
    can_frame frame;
    my_can_signed msg;  
    ID recv_id;
    ID write_id;
    float own_c;
    own_c = own_node.c[own_node.index];
    if(occ)
    {
      LUX_ref = Lbound;
    }
    else
    {
      LUX_ref = lbound;
    }
    delete[] own_node.d;
    delete[] own_node.d_av;
    delete[] own_node.y;
    delete[] own_node.c;
    delete[] own_node.next_it;

    own_node = c_node(ID_ARRAY_LENGHT, ID_IN_ARRAY, Karray, own_c, own_o,  LUX_ref, 0.07);

    while(a < 50)
      {
        own_node.consensus_iterate();
        for(int i = 0; i < ID_ARRAY_LENGHT; i++)
        {
          write_id.bytes[0] = ID_IN_ARRAY;
          write_id.bytes[1] = i;

          write(write_id.value, own_node.d[i]*1000);
          Serial.print("\t");
          Serial.print(own_node.d[i]);
          delay(40);
        }
        Serial.println();
        counter = 0; timeout = 0;

        for(int i = 0; i<ID_ARRAY_LENGHT; i++) 
        {
          prev_dav[i] = own_node.d_av[i];
          own_node.d_av[i] = own_node.d[i];
        }

        while(counter < ID_ARRAY_LENGHT*(ID_ARRAY_LENGHT-1))
        {
          if(interrupt) 
          {

            interrupt = false;
            if (mcp2515_overflow) {
              Serial.println("\t\tError: MCP2515 RX Buffers Full");
              mcp2515_overflow = false;
            }

            if (arduino_overflow) {
              Serial.println("\t\tError: Arduino Stream Buffers Overflow");
              arduino_overflow = false;
            }


            while (cf_stream.get(frame)) 
            {
              for (int i = 0; i < 4; i++) msg.bytes[i] = frame.data[i];
              Serial.print("\t Receiving D:");
              counter++;
              Serial.print((float)msg.value/1000);
              recv_id.value = frame.can_id;
              Serial.print("\t From id in array:");// ta bonit
              Serial.println(recv_id.bytes[1]);
              own_node.d_av[recv_id.bytes[1]] += (float)msg.value/1000;
            }


          }
          timeout++;
          if(timeout == 50)
          {
            Serial.println("TIMEOUT");
            break;
          }
        }
        if (timeout == 50)
        {
          for (int i = 0; i < ID_ARRAY_LENGHT; i++)
            own_node.d_av[i] =  prev_dav[i];
        }
        else
        {  
          for (int i = 0; i < ID_ARRAY_LENGHT; i++)
            own_node.d_av[i] =  (own_node.d_av[i]) / (float)ID_ARRAY_LENGHT;
        }
        
        own_node.update_lagrange();
        Serial.println("d_av ARRAY IS:");
        for(int i = 0; i < ID_ARRAY_LENGHT; i++)
        {
          Serial.println(own_node.d_av[i]);
        }
        delay(500);
        a++;
      }
  ti = millis();
  N = 1;
  Vis_r_all = 0;
  fi = 0;
  t_ant = millis();
  Ref_Lux = own_node.d_av[ID_IN_ARRAY]*Karray[ID_IN_ARRAY]+own_o;
  dcc = set_occ(occ, Ref_Lux);
  delay(200);
}

void setup()
{
  Serial.begin(115200);
  SPI.begin();
  attachInterrupt(digitalPinToInterrupt(2), irqHandler, FALLING); //use interrupt at pin 2
  pinMode(ledPin, OUTPUT);
  TCCR2B = TCCR2B & B11111000 | B00000001;
  // Must SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();
  delay(2000);
  int a =0;
  ID write_id, recv_id;
  my_can_signed msg;
  can_frame frame;
  auto* K_max_lux = new float[ID_ARRAY_LENGHT]; // DONT FORGET TO DEALLOCATE (IF NEEDED)
  Serial.print("own id = ");
  Serial.println(own_id);
  local_id_list.addid(own_id); //Adds its own id to the local id list

  build_ID_list();

  int* array = local_id_list.get_sorted_array();
  int lenght = local_id_list.get_lenght();
  for(int i = 0; i < lenght; i++)
  {
    Serial.println((unsigned char) array[i]);
  }
  ID_ARRAY = array;
  ID_ARRAY_LENGHT = lenght;
  delay(100);
  Karray = init_sync();//each one tries to communicate with each other
  delay(1000);
  OUT_DIST = analogRead(analogPin);
  Serial.println("K ARRAY IS:");
  
  for(int i = 0; i< ID_ARRAY_LENGHT; i++) 
  {
    Karray[i] = volttolux(Karray[i]);
    Serial.println(Karray[i]);
  }

  K_max_lux[ID_IN_ARRAY] = Karray[ID_IN_ARRAY];
  write_id.bytes[0] = 0;
  write_id.bytes[1] = ID_IN_ARRAY;

  if( write(write_id.value, K_max_lux[ID_IN_ARRAY]) != MCP2515::ERROR_OK)
  {
    Serial.println("\t\tError: MCP2515 TX Buffers Full");
  }

  Serial.println("\t\tSent K\t");

  delay(500);

  while(a<ID_ARRAY_LENGHT-1)
  {
    if(interrupt)
    {
      interrupt = false;
      if (mcp2515_overflow) {
        Serial.println("\t\tError: MCP2515 RX Buffers Full");
        mcp2515_overflow = false;
      }

      if (arduino_overflow) {
        Serial.println("\t\tError: Arduino Stream Buffers Overflow");
        arduino_overflow = false;
      }
      while (cf_stream.get(frame)) 
      {
        for(int i=0; i<4; i++) msg.bytes[i]=frame.data[i];
        recv_id.value = frame.can_id;
        Serial.print("Saving:\t");
        Serial.println(msg.value);
        Serial.print("in:\t");
        Serial.println(recv_id.bytes[1]);

        K_max_lux[recv_id.bytes[1]] = msg.value; 
        a++;
      }

    }

    delay(100);
  }
  
  Serial.println("K_max_lux:");
  for(int i = 0; i< ID_ARRAY_LENGHT; i++) 
  {
    Serial.println(K_max_lux[i]);
  }

  own_o = map(OUT_DIST, 0, 1023, 0, 5000);
  own_o = volttolux(own_o);
  Ext_Lux = own_o;

}

void loop()
{
  char client_command[6]; 
  String input_string;
  ID write_id;
  my_can_signed msg;
  can_frame clearframe;
  return_type data;
  int prev_dav[ID_ARRAY_LENGHT];
  // p/ calcular energia

  float t_ant = 0.0;  
  float LUX_curr;
  float  fi =0, Vis_r_all=0;
  int N=1;
  float l_at[3] = {0, 0, 0};
  float ts[3] = {0, 0, 0};
  int volt;
  int auxim;

  ID recv_id;
  can_frame frame;
  Serial.print("\t O:");
  Serial.println(own_o);

  LUX_ref = 0.8*Karray[ID_IN_ARRAY];
  Lbound = LUX_ref;
  lbound = 0.2*Karray[ID_IN_ARRAY];
  c_node own_node(ID_ARRAY_LENGHT, ID_IN_ARRAY, Karray, 1, own_o,  LUX_ref, 0.07);
  int a, timeout =0 ;

  
  Serial.println(ID_IN_ARRAY);
  if(CONSENSUS_FLAG)
  {
    CONSENSUS_FLAG = 0;
    consensus_algorithm(own_node);
  }
  Serial.println("Consensus Done");
  while(1)
  {


    analogWrite(ledPin,(int)map(dcc, 0, 100, 0,255));
    volt = analogRead(analogPin);
    volt = map(volt, 0, 1023, 0, 5000);
    simret = simulation(R2, vf, vi, tau, (millis()-ti));
    val = volt;
    LUX_curr = volttolux(volt);
    


    for(int i=2;i!=0;i--)
    {
      l_at[i] = l_at[i-1]; 
    }
    l_at[0] = LUX_curr;
    ts[0] = millis();

    if((l_at[2]-l_at[1])*(l_at[1]-l_at[0]) >= 0)
    {
      fi = fi +  (abs(l_at[2]-l_at[1])+abs(l_at[1]-l_at[0]))/0.02;
    }
      
    if((volttolux(simret*1000)-LUX_curr)>0)
    {
      Vis_r_all = Vis_r_all + (volttolux(simret*1000)-LUX_curr);
    }

    if((fi < 0) || (Vis_r_all) < 0 || (Vis_r < 0))
    {
      fi = 0;
      Vis_r_all = 0;
      Vis_r = 0;
      N = 1;
    }


    Flic_e = fi/N;
    Vis_r = Vis_r_all/N;   
    energy_spent = energy_spent + dcc*(((millis()-t_ant))/1000);
    
    dcc = pid(simret*1000, val) + dccref;
    t_ant = millis();
    N++;

    if(dcc > 100)
    {
      dcc = 100;
    }
    if (dcc < 0)
    {
      dcc = 0;
    }
    if (Serial.available() > 0)
    {
      input_string = Serial.readString();
      for(int i = 0; i < 9; i++)
      {
        client_command[i] = input_string[i];
      }

      Serial.println("I received:");
      Serial.println(client_command);
      
      write_id.bytes[0]=127;
      write_id.bytes[1]=127;

      for (int i = 0; i < 3; i++)
      {
        msg.bytes[i]=client_command[i];
      }
     
      if((client_command[0] == 'U')||(client_command[0] == 'O'))
      {
        auxim = 8;
        while(client_command[auxim] == NULL)
          auxim--;
        
        if(auxim == 3)
          msg.bytes[3] = client_command[3];
        
        else if(auxim == 4)
        {
          auxim = ((int) client_command[3]-48)*10+((int) client_command[4]-48);
          msg.bytes[3] = (char)auxim;
        }
        else if(auxim == 5)
        {
          auxim = ((int) client_command[3]-48)*100+((int) client_command[4]-48)*10+((int) client_command[5]-48);
          msg.bytes[3] = (char)auxim;
        }
          
      } 
     
      write(write_id.value, msg.value);
      if(client_command[0] == 'o')
      {
        data = GetServerResponse(client_command[0], client_command[1], client_command[2], own_node);
      }
      else if(client_command[0] == 'r'|| client_command[2] - 48 == own_id || client_command[2] > 57)
      {
        data = GetServerResponse(client_command[0], client_command[1], client_command[2], own_node);
        Serial.println(data.decimal);
        Serial.println("OWN");
      }
      else if((client_command[0] == 'U')||(client_command[0] == 'O'))
      {
        data = GetServerResponse(client_command[0], client_command[1], msg.bytes[3], own_node);
        Serial.println(data.decimal);
      } 

      if(CONSENSUS_FLAG)
      {
        delay(300);
        pseudo_setup();
 
        CONSENSUS_FLAG = 0;
        consensus_algorithm(own_node);
      }
      if(OCC_FLAG)
      {
        delay(200);
        OCC_FLAG = 0;
        consensus_algorithm(own_node);
      }

      delay(100);
    }


    if(interrupt)
    {
      
      interrupt = false;
      if (mcp2515_overflow) {
        Serial.println("\t\tError: MCP2515 RX Buffers Full");
        mcp2515_overflow = false;
      }

      if (arduino_overflow) {
        Serial.println("\t\tError: Arduino Stream Buffers Overflow");
        arduino_overflow = false;
      }
      while (cf_stream.get(frame)) 
      {
        for (int i = 0; i < 4; i++) msg.bytes[i] = frame.data[i];
        recv_id.value = frame.can_id;
        
        if (recv_id.bytes[0] != 127)
        {
          Serial.print(recv_id.bytes[0]);
          Serial.print("\t");
          Serial.println(msg.value/1000);
        }
        
        if ((msg.bytes[2] - 48  == own_id)||(msg.bytes[0] == 114)||(msg.bytes[0] == 111))
        {
          Serial.println("Received:");
          Serial.print((char)msg.bytes[0]);
          Serial.print("\t");
          Serial.print((char)msg.bytes[1]);
          Serial.print("\t");
          Serial.print((char)msg.bytes[2]);
          Serial.print("\t");
          Serial.println((char)msg.bytes[3]);
          recv_id.bytes[1] = own_id; // send to the hubduino
          recv_id.bytes[0] = msg.bytes[0]; // tell the hubduino that the request is done
          data = GetServerResponse((char) msg.bytes[0],(char) msg.bytes[1], (char)msg.bytes[2], own_node);
          
          msg.value = data.decimal*1000;
          Serial.println(msg.value);
          write(recv_id.value, msg.value);
        

        }
        if((msg.bytes[0] == 79 )||(msg.bytes[0] == 85))
        {
          Serial.println("Received:");
          Serial.print((char)msg.bytes[0]);
          Serial.print("\t");
          Serial.print((char)msg.bytes[1]);
          Serial.print("\t");
          Serial.println((char)msg.bytes[3]);
          recv_id.bytes[1] = own_id; // send to the hubduino
          recv_id.bytes[0] = msg.bytes[0]; // tell the hubduino that the request is done
          data = GetServerResponse((char) msg.bytes[0],(char) msg.bytes[1], (char)msg.bytes[3], own_node);
          
          msg.value = data.decimal*1000;
          Serial.println(msg.value);
          write(recv_id.value, msg.value);
        }    
        if(CONSENSUS_FLAG)
        {
          pseudo_setup();
          CONSENSUS_FLAG = 0;
          consensus_algorithm(own_node);
          fi = 0;
          Vis_r_all = 0;
          Vis_r = 0;
          N = 1;
        }
        if(OCC_FLAG)
        {
          delay(200);
          OCC_FLAG = 0;
          consensus_algorithm(own_node);
        }
      }
     } 
  }
}