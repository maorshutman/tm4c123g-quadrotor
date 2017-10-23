#include <stdint.h>
#include "buffer.h"

char buff[DATA_LENGTH];
uint8_t index = 0;


//*****************************************************************************
//
//!
//!
//! \param
//! \param
//!
//! ???
//!
//! \return
//
//*****************************************************************************
void WriteByteToBuffer(char ch)
{
    buff[index] = ch;
    index++;
    if (index == DATA_LENGTH)
        index = 0;
}
