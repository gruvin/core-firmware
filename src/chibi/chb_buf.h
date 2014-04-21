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
#ifndef CHB_BUF_H
#define CHB_BUF_H

#include "application.h"
#include "chb_types.h"

#if (CHIBI_PROMISCUOUS)
    // if we're using promiscuous mode, we may end up capturing a lot of frames.
    // crank up the buffer size to handle traffic spikes.
    #define CHB_BUF_SZ 768
#else
    // in normal mode, this is the buffer size to handle incoming frames. if there
    // is a lot of traffic and you're getting buffer issues, then increase this 
    // value so that more frames can be held inside RAM
    #define CHB_BUF_SZ 256
#endif

void chb_buf_init();
void chb_buf_write(U8 data);
U8 chb_buf_read();
U16 chb_buf_get_len();

#endif
