
#pragma once


//! Simple container of a CAN message
class CANMsg {
 public:
  char flag;
  char rtr;
  int COBId;
  unsigned char length;
  unsigned char value[8];

  CANMsg()  {
    flag = 0;
    rtr = 0;
    COBId = 0;
    length = 0;
    for (int i=0; i<8; i++) {
      value[i] = 0;
    }
  }
};

