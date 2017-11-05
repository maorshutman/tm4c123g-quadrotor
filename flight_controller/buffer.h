#ifndef _BUFFERH_
#define _BUFFERH_

#define DATA_LENGTH 14

extern char buff[DATA_LENGTH];
extern uint8_t index;

void WriteByteToBuffer(char ch);

#endif
