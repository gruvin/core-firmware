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
 
    This is the user configurable file. It contains parameters that users
    can change to suit their needs. 

*/
/**************************************************************************/
#ifndef CHB_USR_CFG_H
#define CHB_USR_CFG_H

/**************************************************************************/
/*!
    Specify use of hardware interrupt
*/
/**************************************************************************/

#define USE_INTERRUPT  1
#define CHB_INT_PIN    A1

/**************************************************************************/
/*!
    Enable the chibiArduino stack to run in promiscuous mode. This should
    only be used to analyze raw packet frames, like for wireshark. 
*/
/**************************************************************************/
#define CHIBI_PROMISCUOUS 0

/**************************************************************************/
/*!
    Max payload determines the largest size that can be transmitted in a single frame.
    If a frame is transmitted that's greater than the max payload, then it
    will be split up into the max payload size and transmitted over multiple frames.

        Integer, byte(s); Range: 1 - 116; Default: 100
        NOTE values over 100 may not work. Please see the documentation.
    
*/
/**************************************************************************/
#define CHB_MAX_PAYLOAD   100

/**************************************************************************/
/*!
    This is the position in EEPROM where the 64-bit IEEE address is stored.
    It takes up 8 byte positions in the EEPROM. 

        HEX, EEPROM location; Range: 0x00 - 0x01F8; Default: 0x00
*/
/**************************************************************************/
#define CHB_EEPROM_IEEE_ADDR    0x00

/**************************************************************************/
/*!
    This is the position in the EEPROM where the 16-bit short address is
    stored. It takes up 2 bytes in the EEPROM.
    
        HEX, EEPROM location; Range: 0x00 - 0x1FE; Default 0x09
*/
/**************************************************************************/
#define CHB_EEPROM_SHORT_ADDR   0x09

/**************************************************************************/
/*!
    This is where the SLP_TR pin is defined. 
*/
/**************************************************************************/
#define CHB_SLPTR_PIN        A0

/**************************************************************************/
/*!
    This is where the SPI Chip Select pin is defined.
*/
#define CHB_SPI_CS_PIN A2


/**************************************************************************/
/*!
    This is where the interrupt configuration code is. This may be different
    based on the chip or type of interrupt being used. 
*/
/**************************************************************************/    
// enable rising edge interrupt on IRQ0
#if (USE_INTERRUPT == 1)
    #define CHB_RADIO_IRQ       rxInterruptHandler
    #define CFG_CHB_INT() do                                          \
                {                                                     \
                  pinMode(CHB_INT_PIN, INPUT_PULLUP);                 \
                  attachInterrupt(CHB_INT_PIN,                       \
                                   CHB_RADIO_IRQ,                     \
                                   RISING);                           \
                }                                                     \
                while(0)
#endif

/**************************************************************************/
/*!
    This is the code to enable and disable the interrupts on the MCU side.
    This is only used when we're in RX_POLLING_MODE where the interrupt needs
    to be turned off on the MCU side until the data can be retrieved. It also
    is a workaround for a nasty bug on the Wiznet W5100 chips where the
    SPI interface is not released properly. Hence, the interrupts are turned
    off until the SPI bus is free and the data can be retrieved without collision.
*/
/**************************************************************************/
//TODO This AVR code needs changing to sjuit STM32 for polling mode
#if (USE_INTERRUPT == 1)
    #define CHB_IRQ_DISABLE() do {PCMSK0 &= ~_BV(PCINT6);} while(0)
    #define CHB_IRQ_ENABLE() do {PCMSK0 |= _BV(PCINT6);} while(0)
#else
    #define CHB_IRQ_DISABLE() do {EIMSK &= ~_BV(INT1);} while(0)
    #define CHB_IRQ_ENABLE() do {EIMSK |= _BV(INT1);} while(0)
#endif

/**************************************************************************/
/*!
    The default channel for the radio to start on.
 
    For 802.15.4 in the 868/915 MHz band, the channels go from 0 to 10. Channel
    0 is 868.3 MHz and is the only channel in the 900 MHz band that can be used
    license free in Europe. Channels 1 through 10 are in the 915 MHz band and
    can be used license free in most of North America. These channels require a
    868/915 MHz radio.
 
            Channel     Frequency (MHz)
            0           868.3
            1           906
            2           908
            3           910
            4           912
            5           914
            6           916
            7           918
            8           920
            9           922
            10          924
 
    For 802.15.4 in the 2.4 GHz band, the channels go from 11 to 26. Channels that
    don't conflict with 802.11 (Wi-Fi) are channels 15, 20, and 26. These channels are
    license free worldwide. They also require a 2.4 GHz radio.
 
            Channel     Frequency (MHz)
            11          2405
            12          2410
            13          2415
            14          2420
            15          2425
            16          2430
            17          2435
            18          2440
            19          2445
            20          2450
            21          2455
            22          2460
            23          2465
            24          2470
            25          2475
            26          2480
*/
/**************************************************************************/
#define CHB_2_4GHZ_DEFAULT_CHANNEL     11
#define CHB_900MHZ_DEFAULT_CHANNEL     1

/**************************************************************************/
/*!
    This is the default modulation mode for the 900 MHz boards using the
    AT86RF212. 
*/
/**************************************************************************/
#define CHB_INIT_MODE OQPSK_SIN    

/**************************************************************************/
/*!
    This is the default PAN ID (network ID) of all devices on the network.
    Only devices sharing the same PAN ID can communicate with each other,
    unless the radio is set to promiscuous mode. The PAN ID is a 16-bit value.
    
        HEX; Range: 0x0000 - 0xFFFF; Default: 0x1234
*/
/**************************************************************************/
#define CHB_PAN_ID  0x1234

/**************************************************************************/
/*!
    This is the value that gets written into the radio's register. Each value
    corresponds to a different transmit power in dBm. The mapping is as follows:
*/
/**************************************************************************/
#define CHB_2_4GHZ_TX_PWR  0x0
#define CHB_900MHZ_TX_PWR  0xE1

/**************************************************************************/
/*!
    This is the number of times the radio will auto-retry a transmission.
    The auto-retry is triggered when an ACK isn't received within the ACK
    timeout period. This is typically ~1 msec. 
    
        Integer, count; Range: 0 - 15; Default: 3
*/
/**************************************************************************/
#define CHB_MAX_FRAME_RETRIES   3

/**************************************************************************/
/*!
    This is the number of times the radio will attempt a transmission when
    the channel is busy. The radio first checks if the channel is clear.
    If it's occupied by another node, then it will back-off for a random
    period and try again. If it exceeds the MAX CSMA RETRIES, it will assume
    the channel is blocked and return an error status.
    
        Integer, count; Range: 0 - 9; Default: 4
*/
/**************************************************************************/
#define CHB_MAX_CSMA_RETRIES    4

/**************************************************************************/
/*!
    This is the minimum backoff exponent. When the channel is busy, the radio
    will wait at least 2^MIN_BE symbol periods before another attempt at
    transmission.
    
        Integer, exponent; Range: 0 - 3; Default: 0
*/
/**************************************************************************/
#define CHB_MIN_BE  0

/**************************************************************************/
/*!
    This is the clear channel assessment mode used to determine whether a
    channel is clear or not. There are generally two ways to assess a busy
    channel. One is to do an energy detection by sampling the energy level
    of the channel. If its below the threshold, then its clear. The other
    way is to check for a 2.4 GHz carrier signal.
 
        Value       Mode
        0           Reserved
        1           Mode 1, Energy above threshold      (Default)
        2           Mode 2, Carrier sense only
        3           Mode 3, Carrier sense with energy above threshold 
*/
/**************************************************************************/
#define CHB_CCA_MODE    0x1

/**************************************************************************/
/*!
    This is the energy detection threshold for the clear channel assessment.
    A channel is considered busy when the energy in the channel is:
    RSSI_BASE_VAL + 2 � CCA_ED_THRES [dBm]
 
    where RSSI_BASE_VAL = -91 dBm for the AT86RF230
    
        Integer, byte; Range: 0x01 - 0x0F; Default: 0x07
*/
/**************************************************************************/
#define CHB_CCA_ED_THRES    0x7

#endif
