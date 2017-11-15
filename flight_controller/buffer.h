#ifndef _BUFFERH_
#define _BUFFERH_

#define PACKET_LENGTH 14

extern char prevPacket[PACKET_LENGTH];
extern char buff[PACKET_LENGTH];
extern uint8_t writeCharIndex;
extern uint32_t prevPacketIndex;
extern uint32_t currPacketIndex;

void WriteByteToBuffer(char ch);
char getCharFromBuffer(uint8_t i);

#endif
