#include <stdint.h>
#include "buffer.h"

char prevPacketBuffer[PACKET_LENGTH];
char buff[PACKET_LENGTH];
uint8_t writeCharIndex = 0;
uint32_t prevPacketIndex = 0;
uint32_t currPacketIndex = 0;

//*****************************************************************************
//
// Writes a single byte into the buffer.
//
//*****************************************************************************
void WriteByteToBuffer(char ch)
{
    buff[writeCharIndex] = ch;
    writeCharIndex++;
    if (writeCharIndex == PACKET_LENGTH)
    {
        writeCharIndex = 0;
    }
}


//*****************************************************************************
//
// Returns a byte from the buffer.
//
//*****************************************************************************
char
GetCharFromBuffer(uint8_t i)
{
    return prevPacketBuffer[i];
}
