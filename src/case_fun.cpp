#include"case_fun.h"
#include "defs.h"


return_type GetServerResponse(char oper, char arg, char tag, c_node own_node)
{
  return_type ret;
  // Serial.println(oper);
  // Serial.println(tag);
  // Serial.println(arg);
  // Serial.println(tag);
  
  if (oper == 'g') // if the first character is a "g"
  {
     switch(arg) // switches based on the middle character
     {
      case 'l': // if its an "l" it gets the illuminance (floating point)
        ret.decimal = volttolux(val);
        return ret;
        break;

      case 'd': // if its an "d" it gets the duty cycle (floating point)
        ret.decimal = dcc;
        return ret;
        break;

      case 'o': // if its an "o" it gets the state of the desk (bool)
        ret.decimal = (float) occ;
        return ret;
        break;

      case 'O': // if its an "O" it gets the lower bound for the occupied state (lux, floating point)
        ret.decimal = Lbound;
        return ret;
        break;
      case 'U': // if its an "U" it gets the lower bound for the unoccupied state (lux,float)
        ret.decimal = lbound;
        return ret;
        break;
      case 'L': // if its an "L" it gets illuminance lower bound (lux, float)
        if(occ) 
        {
          ret.decimal = Lbound;
        }
        else
        {
          ret.decimal = lbound;
        }
        return ret;
        break;
      case 'x': // external illuminace (float, lux)
        ret.decimal = Ext_Lux;
        return ret;
        break;
      case 'r': // illuminance control reference (float, lux)
        ret.decimal = Ref_Lux;
        return ret;
        break;
      case 'c': // current cost of energy (float, joule)
        ret.decimal = own_node.c[own_node.index];
        return ret;
        break;
      case 'p': // instantaneous power at desk (float, watt)
        ret.decimal = dcc/100;
        return ret;
        break;
      case 't': // elapsed time in seconds (float)
        ret.decimal = (startt - millis())/1000;
        return ret;
        break;
      case 'e': // accumulated energy comsuption (float, joule)
        ret.decimal = energy_spent;
        return ret;
        break;
      case 'v': // accumulated visibility error (float, lux)
        ret.decimal = Vis_r;
        return ret;
        break;
      case 'f': // accumulated flicker error (float, lux/s)
        ret.decimal = Flic_e;
        return ret;
        break;
     }
    if (tag == 'T') // Total metrics
    {
      switch(arg)
      {
        case 'p': // total instantaneous  power (float, watt) 
          ret.decimal = dcc/100;
          return ret; 
          break;

        case 'e': // total accumulated energy (float, joule)
          ret.decimal = energy_spent;
          return ret;
          break;

        case 'v': // total visibility error (float, lux)
          ret.decimal = Vis_r;
          return ret;
          break;

        case 'f': // total accumulated flicker error (float, lux/s)
          ret.decimal = Flic_e;
          return ret;        
          break;
      }
      
    }  
  }
  if (oper == 'o') // set occupcancy state
  {
    if(tag - 48 == own_node.index+1)
    {
      // occ = (bool) arg - 48;
      Serial.print("ARG IS \t");
      Serial.println(arg-48);
      if(arg != '0')
      {
        LUX_ref = Lbound;
        occ = true;
      }
      else
      {
        LUX_ref = lbound;
        occ = false;
      }

    }
      OCC_FLAG = 1;
      ret.decimal = 1;
      return ret;
    // ID write_id;
    // my_can_msg msg;
    // write_id.bytes[0] = 200;
    // write_id.bytes[1] = 200;
    // msg.bytes[0] = 46;
    // write(write_id.value, msg.value);

  }
  if (oper == 'O') // set lower bound for occupied state
  {
    if(arg - 48 == own_node.index+1)
    {
      Lbound = tag;
    }
    OCC_FLAG = 1;
    ret.decimal = 1;
    return ret;
  }
  if (oper == 'U') // set lower bound for unoccupied state
  {
    if(arg - 48 == own_node.index+1)
    {
      lbound = tag;
    }
    OCC_FLAG = 1;
    ret.decimal = 1;
    return ret;
  }
  if (oper == 'c') // set current energy cost
  {
    
    own_node.c[own_node.index] = tag - 48;
    ret.decimal = 1;
    return ret;
  }
  if (oper == 'r') // restart system (reset and recalibrate)
  {
    CONSENSUS_FLAG = 1;
    ret.decimal = 50000;
    delay(200);
    ret.decimal = 1;
    return ret;
  }
  // if (oper == 'b') // get last minute buffer of variable
  // {
  //   if (tag == tag)
  //   {
  //     switch(arg)
  //     {
  //       case 'I':
  //         break;

  //       case 'd':
  //         break;
  //     }
  //   }
  // }
  if (oper == 'S') // start stream of real-time variable 
  {
    
    if (tag == tag)
    {
      switch(arg)
      {
        case 'I':
        Serial.println("lolada, apa soce");
          break;

        case 'd':
          break;
      }
    }
  }
    if (oper == 's') // stop stream of real-time variable
  {
    
    if (tag == tag)
    {
      switch(arg)
      {
        case 'I':
          break;

        case 'd':
          break;
      }
    }
  }
}