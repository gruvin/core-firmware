/*******************************************************************
    Copyright (C) 2009 FreakLabs
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.

    Originally written by Christopher Wang aka Akiba.
    Please post support questions to the FreakLabs forum.

*******************************************************************/
/*!
    \file 
    \ingroup


*/
/**************************************************************************/
#include "chb_eeprom.h"

#include "application.h"

/*******************************************************************************************/
/* TEMPOARY SOLUTION uplifted from https://community.spark.io/t/lets-get-non-volatile/2200 */
// XXX: We'll x2 all 'EEPROM' addresses as an ugly hack to make this work. See forum notes.
int SparkFlash_read(int address)
{
    if (address & 1)
          return -1; // error, can only access half words

      uint8_t values[2];
        sFLASH_ReadBuffer(values, 0x80000 + address, 2);
          return (values[0] << 8) | values[1];
}

int SparkFlash_write(int address, uint16_t value)
{
    if (address & 1)
          return -1; // error, can only access half words

      uint8_t values[2] = {
            (uint8_t)((value >> 8) & 0xff),
                (uint8_t)(value & 0xff)
                    };
        sFLASH_WriteBuffer(values, 0x80000 + address, 2);
          return 2; // or anything else signifying it worked
}
/*******************************************************************************************/


/**************************************************************************/
/*!
  Write to Flash as EEPROM emmulation.
*/
/**************************************************************************/
void chb_eeprom_write(U16 addr, U8 *buf, U16 size)
{
    if (size > 256) return; // Restrict to 1/4 the available Flash page size
    
    for (U8 i=0; i < size; i++)
    {
      SparkFlash_write((addr+i)*2/*XXX*/, buf[i]);
    }
}

/**************************************************************************/
/*!
  Read from Flash as EEPROM emmulation.
*/
/**************************************************************************/
void chb_eeprom_read(U16 addr, U8 *buf, U16 size)
{
    U16 data;
    if (size > 256) return; // Restrict to 1/4 the available Flash page size

    for (U8 i=0; i < size; i++)
    {
      data = SparkFlash_read((addr+i)*2/*XXX*/);
      buf[i] = (U8)data;
    }
}
