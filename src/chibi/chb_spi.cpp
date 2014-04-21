/*******************************************************************

   Port of Freaklabs chibiArduino to STM32 Sparkcore 
   by Bryan J. Rentoul aka Gruvin.

Original Author copyright and licence information ...

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
   3. Neither the name of the the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from this
      software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS
   IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
   PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.

   Originally written by Christopher Wang aka Akiba.
   Please post support questions [for the original chibiArduino code ONLY] to the 
   FreakLabs forum.

*******************************************************************/
/*!
    \file 
    \ingroup


*/
/**************************************************************************/
#include "chb.h"
#include "chb_spi.h"

/**************************************************************************/
/*!
    Initialize the SPI
*/
/**************************************************************************/
void chb_spi_init()
{
    // set the SPI slave select to idle
    CHB_SPI_DISABLE(); // CHB_SPI_CS_PIN to high

    SPI.setClockDivider(SPI_CLOCK_DIV64);
    SPI.setDataMode(0);
    SPI.begin(CHB_SPI_CS_PIN); // Standard Spark SPI set-up uses software controlled /SS pin (good).
}

/**************************************************************************/
/*!
    This function both reads and writes data. For write operations, include data
    to be written as argument. For read ops, use dummy data as arg. Returned
    data is read byte val.
*/
/**************************************************************************/
U8 chb_xfer_byte(U8 data)
{
  /*
    while (!(SPI1_BASE->SR & SPI_SR_TXE));  // wait for TX buffer to become available
    SPI1_BASE->DR = data;
    while (!(SPI1_BASE->SR & SPI_SR_RXNE)); // wait for "RX buffer not empty" flag to go high
    return SPI1_BASE->DR;
    */
  return SPI.transfer(data);
}
