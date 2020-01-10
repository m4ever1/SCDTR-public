#ifndef CASEF_H
#define CASEF_H
#include <string.h>
#include <Arduino.h>
#include "consensus.h"

union return_type
{
  char str[4];
  float decimal;
};

return_type GetServerResponse(char oper, char arg, char tag, c_node own_node);



#endif