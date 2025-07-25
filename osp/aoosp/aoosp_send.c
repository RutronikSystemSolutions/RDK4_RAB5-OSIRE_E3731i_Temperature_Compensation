// aoosp_send.cpp - send command telegrams (and receive response telegrams)
/*****************************************************************************
 * Copyright 2024,2025 by ams OSRAM AG                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************/

#include "..\aospi\aospi.h"      // aospi_tx, aospi_txrx
#include "aoosp_crc.h"  // aoosp_crc
#include "aoosp_prt.h"  // aoosp_prt_bytes() for logging
#include "aoosp_send.h" // own API

#include <string.h> // memcpy


// Definition of a telegram
#define AOOSP_TELE_MAXSIZE 12
typedef struct aoosp_tele_s {
	uint8_t data[AOOSP_TELE_MAXSIZE];
	uint8_t size;
} aoosp_tele_t;


// Generic telegram field access macros
#define BITS_MASK(n)          ( (1<<(n))-1 )                          // series of n bits: BITS_MASK(3)=0b111 (max n=31)
#define BITS_SLICE(v,lo,hi)   ( ((v)>>(lo)) & BITS_MASK((hi)-(lo)) )  // takes bits [lo..hi) from v: BITS_SLICE(0b11101011,2,6)=0b1010
#define SIZE2PSI(payloadsize) ( (payloadsize)<8 ? (payloadsize) : 7 ) // convert payload size, to PSI (payload size indicator)
#define TELEPSI(tele)         ( BITS_SLICE((tele)->data[1],0,2)*2 + BITS_SLICE((tele)->data[2],7,8) ) // get PSI bits from tele


// === LOG ================================================


// Logging
// =======
// It is mandatory to have the macro AOOSP_LOG_ENABLED defined to 0 or 1.
// When AOOSP_LOG_ENABLED is defined to 0, all logging code is compiled out,
// and the API functions are stubbed to empty.
// When AOOSP_LOG_ENABLED is defined to 1, the level set with function
// aoosp_loglevel_set() determines what is printed to Serial when calling
// aoosp_send_xxx() functions.
// When telegram arguments are printed, there is one issue: the logger does
// not know whether the telegram comes from a SAID or an RGBi, and for
// some telegram arguments the meaning depends on that. In this case,
// the logger will print the SAID meaning first, and then, between brackets,
// the RGBi meaning.
// For example, the temperature and status reported by init differ for
// SAID and RGBi, the log solves this as follows
//   initloop(0x001)
//     [tele A0 04 03 86] -> [resp A0 09 03 00 50 63]
//     last=0x02=2 temp=0x00=-86 stat=0x50=SLEEP:tV:clou (-126, SLEEP:oL:clou)


#ifndef AOOSP_LOG_ENABLED
  #error Macro AOOSP_LOG_ENABLED must be defined
#else
  #if AOOSP_LOG_ENABLED!=0 && AOOSP_LOG_ENABLED !=1
    #error Macro AOOSP_LOG_ENABLED must be 0 or 1
  #endif
#endif


#if AOOSP_LOG_ENABLED

static aoosp_loglevel_t aoosp_loglevel = aoosp_loglevel_none;


/*!
    @brief  Sets the amount of logging for the aoosp_send_xxx() functions.
    @param  level
            aoosp_loglevel_none - Nothing is logged (default)
            aoosp_loglevel_args - Logging of sent and received telegram arguments
            aoosp_loglevel_tele - Also logs raw (sent and received) telegram bytes
    @note   Logging means "print to Serial".
    @note   If arguments are logged, by default the SAID interpretation is shown,
            the RGBI interpretation is appended in parenthesis.
    @note   Current log level is observable via aoosp_loglevel_get().
    @note   Default log level is aoosp_loglevel_none.
*/
void aoosp_loglevel_set(aoosp_loglevel_t level) {
  aoosp_loglevel = level;
}


/*!
    @brief  Gets the log level (amount of logging) for the aoosp_send_xxx() functions.
    @return Active log level
    @note   See aoosp_loglevel_set() for details.
*/
aoosp_loglevel_t aoosp_loglevel_get() {
  return aoosp_loglevel;
}

#endif // AOOSP_LOG_ENABLED


// === TELEGRAMS ==========================================


// Construct, destruct and send
// ============================
// Per telegram ID there are three functions.
// Let the telegram name (ID) be xxx, argx the arguments and resx the results in the response.
//
//
// The first function constructs a telegram, eg converts (int) arguments to a byte array.
//   static aoresult_t aoosp_con_xxx(aoosp_tele_t * tele, uint16_t addr, uintx_t arg0, uintx_t arg1, ... , uint8_t * respsize)
//
//   - aoosp_tele_t * tele   (OUT) caller allocated (content irrelevant), filled with preamble/addr/tid/args/aoosp_crc on exit
//   - uint16_t addr         (IN)  is the address of the destination node (unicast), or 0 (broadcast), or a group address (3F0..2FE)
//   - uintx_t arg0          (IN)  telegram xxx specific argument
//   - uintx_t arg1          (IN)  telegram xxx specific argument
//   - ...                   (IN)  ...
//   - uint8_t * respsize    (OUT) if not NULL set to expected response size (telegram size, so payload size plus 4, in bytes)
//                                 telegrams without a response do not have this out parameter
//
//   - returns aoresult_ok if all ok, or aoresult_osp_arg, aoresult_osp_addr or aoresult_outargnull.
//
//
// The second function destructs a (response) telegram, e.g. converts a byte array to (int) result fields
//   static aoresult_t aoosp_des_xxx(aoosp_tele_t * tele, uintx_t * res0, uintx_t * res1, ... )
//   - aoosp_tele_t * tele   (IN)  caller allocated, checked for correct telegram id, size and CRC
//
//   - uintx_t * res0        (OUT) set to response field (telegram xxx specific response)
//   - uintx_t * res1        (OUT) set to response field (telegram xxx specific response)
//   - ...                   (OUT) ...
//
//   - returns aoresult_ok if all ok, or aoresult_osp_preamble, aoresult_osp_tid, aoresult_osp_size, aoresult_osp_crc, or aoresult_outargnull.
//
//
// The third function is a helper.
//   aoresult_t aoosp_send_xx(...)
//   - has all payload arguments of aoosp_con_xxx, then has all result fields of aoosp_des_xxx
//     in other words, all arguments of both functions with the exception of tele, respsize, tele
//   - constructs a telegram, sends it, waits for response (if there is one), destructs it.
//   - optionally prints logging info
//   - returns aoresult_ok if all ok, or any of the constructor or destructor errors, or aoresult_spi_xxx.


// Result handling
// ===============
// The telegram send function consist of three (or two) steps:
// construct, send (or send and receive), and destruct (if there was a receive).
//
// The chosen style (to ease logging) is to have a result per step and one
// accumulated result. All are initialized to "no error":
//   aoresult_t result    = aoresult_ok;
//   aoresult_t con_result= aoresult_ok;
//   aoresult_t spi_result= aoresult_ok;
//   aoresult_t des_result= aoresult_ok;
//
// Next, each step is executed conditionally: only if accumulated
// result indicates no error yet. After each step the accumulated
// result is updated with the step result:
//
//   if(     result==aoresult_ok ) con_result= aoosp_con_xxx(...);
//   if( con_result!=aoresult_ok ) result=con_result;
//
//   if(     result==aoresult_ok ) spi_result= aospi_txrx(...);
//   if( spi_result!=aoresult_ok ) result= spi_result;
//
//   if(     result==aoresult_ok ) des_result = aoosp_des_xxx(...)
//   if( des_result!=aoresult_ok ) result=des_result;


// ==========================================================================
// Telegram 00 RESET


static aoresult_t aoosp_con_reset(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x00; // RESET
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}

/*!
    @brief  Sends a RESET telegram.
            This resets all nodes in the chain (all "off"; they also lose their address).
    @param  addr
            The address to send the telegram to, use 0 (broadcast).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   Node will lose its address (use INITBIDIR/LOOP to reassign).
    @note   Will also reset comms mode (MCU, EOL, LVDS, CAN),
            inspecting the SIO line levels.
    @note   Will not reset P2RAM cache of the OTP.
    @note   Because the execution of a RESET command takes an
            extraordinary amount of time (unlike most other commands),
            wait 150us after sending this telegram.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_reset(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_reset(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("reset(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  };
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 01 CLRERROR


static aoresult_t aoosp_con_clrerror(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x01; // CLRERROR
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a CLRERROR telegram.
            This clears the error flags of the addressed node.
    @param  addr
            The address to send the telegram to (unicast),
            (use 0 for broadcast, or 3F0..3FE for group).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When error flags are set, a node will not go active.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_clrerror(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_clrerror(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("clrerror(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  };
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 02 INITBIDIR


static aoresult_t aoosp_con_initbidir(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x02; // INITBIDIR
  if( respsize ) *respsize = 4+2; // temp, stat

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_initbidir(aoosp_tele_t * tele, uint16_t * last, uint8_t * temp, uint8_t * stat) {
  // Set constants
  const uint8_t payloadsize = 2;
  // Check telegram consistency
  if( tele==0 || last==0 || temp==0 || stat==0 ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x02      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size) != 0    ) return aoresult_osp_crc;

  // Get fields
  *last = BITS_SLICE(tele->data[0],0,4)<<6 | BITS_SLICE(tele->data[1],2,8);
  *temp = tele->data[3];
  *stat = tele->data[4];

  return aoresult_ok;
}


/*!
    @brief  Sends an INITBIDIR telegram and receives its response.
            This assigns an address to each node.
            Also configures all nodes for BiDir - they send response backward.
    @param  addr
            The address to send the telegram to, typically use 1 (serialcast).
    @param  last
            Output parameter returning the address of
            the last node - that is the chain length.
    @param  temp
            Output parameter returning the raw temperature of the last node.
    @param  stat
            Output parameter returning the status of the last node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameters are set.
    @note   Make sure the chain is wired as BiDir; e.g. if you have the OSP32
            board, precede this call with a call to aospi_dirmux_set_bidir().
    @note   If there are branches, send INITBIDIR once for every branch,
            with the start address for that branch.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_initbidir(uint16_t addr, uint16_t * last, uint8_t * temp, uint8_t * stat) {
  // Telegram and result vars
  aoosp_tele_t  tele;
  aoosp_tele_t  resp;
  aoresult_t    result    = aoresult_ok;
  aoresult_t    con_result= aoresult_ok;
  aoresult_t    spi_result= aoresult_ok;
  aoresult_t    des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_initbidir(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_initbidir(&resp, last, temp, stat);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("initbidir(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" last=0x%03X=%d temp=0x%02X=%d stat=0x%02X=%s",  *last, *last,
      *temp, aoosp_prt_temp_said(*temp), *stat, aoosp_prt_stat_said(*stat) );
    Serial.printf(" (%d, %s)\n",  aoosp_prt_temp_rgbi(*temp), aoosp_prt_stat_rgbi(*stat) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 03 INITLOOP


static aoresult_t aoosp_con_initloop(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x03; // INITLOOP
  if( respsize ) *respsize = 4+2; // temp, stat

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_initloop(aoosp_tele_t * tele, uint16_t * last, uint8_t * temp, uint8_t * stat) {
  // Set constants
  const uint8_t payloadsize = 2;
  // Check telegram consistency
  if( tele==0 || last==0 || temp==0 || stat==0 ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x03      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size) != 0    ) return aoresult_osp_crc;

  // Get fields
  *temp = tele->data[3];
  *stat = tele->data[4];
  *last = BITS_SLICE(tele->data[0],0,4)<<6 | BITS_SLICE(tele->data[1],2,8);

  return aoresult_ok;
}


/*!
    @brief  Sends an INITLOOP telegram and receives its response.
            This assigns an address to each node.
            Also configures all nodes for Loop - they send response forward.
    @param  addr
            The address to send the telegram to, typically use 1 (serialcast).
    @param  last
            Output parameter returning the address of
            the last node - that is the chain length.
    @param  temp
            Output parameter returning the raw temperature of the last node.
    @param  stat
            Output parameter returning the status of the last node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameters are set.
    @note   Make sure the chain is wired as Loop; e.g. if you have the OSP32
            board, precede this call with a call to aospi_dirmux_set_loop().
    @note   If there are branches, probably it is better to use INITBIDIR.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_initloop(uint16_t addr, uint16_t * last, uint8_t * temp, uint8_t * stat) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_initloop(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_initloop(&resp, last, temp, stat);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("initloop(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" last=0x%03X=%d temp=0x%02X=%d stat=0x%02X=%s",  *last, *last,
      *temp, aoosp_prt_temp_said(*temp), *stat, aoosp_prt_stat_said(*stat) );
    Serial.printf(" (%d, %s)\n",  aoosp_prt_temp_rgbi(*temp), aoosp_prt_stat_rgbi(*stat) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 04 GOSLEEP


static aoresult_t aoosp_con_gosleep(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x04; // GOSLEEP
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends an GOSLEEP telegram.
            This switches the state of the addressed node to sleep
            (switching off all LEDs).
    @param  addr
            The address to send the telegram to (unicast),
            (use 0 for broadcast, or 3F0..3FE for group).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_gosleep(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_gosleep(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("gosleep(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 05 GOACTIVE


static aoresult_t aoosp_con_goactive(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x05; // GOACTIVE
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends an GOACTIVE telegram.
            This switches the state of the addressed node to active
            (allowing to switch on LEDs).
    @param  addr
            The address to send the telegram to (unicast),
            (use 0 for broadcast, or 3F0..3FE for group).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_goactive(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_goactive(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("goactive(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 06 GODEEPSLEEP


static aoresult_t aoosp_con_godeepsleep(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x06; // GODEEPSLEEP
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends an GODEEPSLEEP telegram.
            This switches the state of the addressed node to deep sleep
            (e.g. LEDs are off).
    @param  addr
            The address to send the telegram to (unicast),
            (use 0 for broadcast, or 3F0..3FE for group).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_godeepsleep(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_godeepsleep(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("godeepsleep(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 07 IDENTIFY


static aoresult_t aoosp_con_identify(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x07; // IDENTIFY
  if( respsize ) *respsize = 4+4; // id

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_identify(aoosp_tele_t * tele, uint32_t * id) {
  // Set constants
  const uint8_t payloadsize = 4;
  // Check telegram consistency
  if( tele==0 || id==0                         ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x07      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *id = (uint32_t)(tele->data[3])<<24 | (uint32_t)(tele->data[4])<<16 | (uint32_t)(tele->data[5])<<8 | (uint32_t)(tele->data[6]);

  return aoresult_ok;
}


/*!
    @brief  Sends an IDENTIFY telegram and receives its response.
            Asks the addressed node to respond with its ID.
    @param  addr
            The address to send the telegram to (unicast).
    @param  id
            Output parameter returning the id of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameter is set.
    @note   See  AOOSP_IDENTIFY_ID2XXX to get the components from the id.
    @note   There is a convenience macro to check for a specific part: e.g.
            AOOSP_IDENTIFY_IS_SAID(id) indicates if the node is a SAID.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_identify(uint16_t addr, uint32_t * id) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_identify(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_identify(&resp, id);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("identify(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" id=0x%08lX\n",*id);
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 08 P4ERRBIDIR
// Telegram 09 P4ERRLOOP


// ==========================================================================
// Telegram 0A ASKTINFO
static aoresult_t aoosp_con_asktinfo(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x0A; // ASKTINFO
  if( respsize ) *respsize = 4+2; //  tmin, tmax

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_asktinfo(aoosp_tele_t * tele, uint8_t * tmin, uint8_t * tmax) {
  // Set constants
  const uint8_t payloadsize = 2;
  // Check telegram consistency
  if( tele==0 || tmin==0 || tmax==0            ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x0A      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *tmax= tele->data[3];
  *tmin= tele->data[4];

  return aoresult_ok;
}


/*!
    @brief  Sends a ASKTINFO telegram and receives its response.
            This is a serial cast accumulating the minimum and maximum
            temperature over the OSP chain (from node `addr` to last one).
    @param  addr
            The address to send the telegram to (serialcast).
    @param  tmin
            Output parameter returning the minimum temperature of the OSP
            chain (from node `addr` to last one).
    @param  tmax
            Output parameter returning the maximum temperature of the OSP
            chain (from node `addr` to last one).
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameters are set.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   Generally, the telegram is sent to the first node (addr=001),
            but is possible to user a higher address.
    @note   Temperature is returned as raw values; use aoosp_prt_temp_rgbi()
            or aoosp_prt_temp_said() to convert to Celsius.
    @note   Different type of node have a different notion of raw temperature.
            As a result, the accumulation with this serial cast only makes
            sense if all nodes have the same type (IDENTIFY).
*/
aoresult_t aoosp_send_asktinfo(uint16_t addr, uint8_t * tmin, uint8_t * tmax) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_asktinfo(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_asktinfo(&resp, tmin, tmax);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("asktinfo(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" tmin=0x%02X=%d tmax=0x%02X=%d", *tmin, aoosp_prt_temp_said(*tmin), *tmax, aoosp_prt_temp_said(*tmax) );
    Serial.printf(" (%d, %d)\n", aoosp_prt_temp_rgbi(*tmin), aoosp_prt_temp_rgbi(*tmax) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 0A ASKTINFO with INIT


static aoresult_t aoosp_con_asktinfo_init(aoosp_tele_t * tele, uint16_t addr, uint8_t tmin, uint8_t tmax, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 2; // initial tmin and tmax
  const uint8_t tid = 0x0A; // ASKTINFO
  if( respsize ) *respsize = 4+2; // tmin, tmax

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = tmax;
  tele->data[4] = tmin;

  tele->data[5] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a ASKTINFO telegram with initial values for tmin and tmax
            and receives its response.
    @param  addr
            The address to send the telegram to (serialcast).
    @param  tmin
            Input parameter passing the initial value for tmin.
            Output parameter returning the minimum temperature of the OSP
            chain (from node `addr` to last one).
    @param  tmax
            Input parameter passing the initial value for tmax.
            Output parameter returning the maximum temperature of the OSP
            chain (from node `addr` to last one).
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameters are set.
    @note   This is the extended version of aoosp_send_asktinfo().
            See there for details.
*/
aoresult_t aoosp_send_asktinfo_init(uint16_t addr, uint8_t * tmin, uint8_t * tmax) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_asktinfo_init(&tele,addr,*tmin,*tmax,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_asktinfo(&resp, tmin, tmax);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("asktinfo_ex(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" tmin=0x%02X=%d tmax=0x%02X=%d", *tmin, aoosp_prt_temp_said(*tmin), *tmax, aoosp_prt_temp_said(*tmax) );
    Serial.printf(" (%d, %d)\n", aoosp_prt_temp_rgbi(*tmin), aoosp_prt_temp_rgbi(*tmax) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 0B ASKVINFO


// ==========================================================================
// Telegram 0C READMULT


static aoresult_t aoosp_con_readmult(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x0C; // READMULT
  if( respsize ) *respsize = 4+2; // groups

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readmult(aoosp_tele_t * tele, uint16_t *groups ) {
  // Set constants
  const uint8_t payloadsize = 2;
  // Check telegram consistency
  if( tele==0 || groups==0                     ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x0C      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *groups= (tele->data[3] << 8) | (tele->data[4] << 0);

  return aoresult_ok;
}


/*!
    @brief  Sends a READMULT telegram and receives its response.
            Asks the addressed node to respond with its mult
            (e.g. a bit mask indicating to which of the 15 groups
            the node belongs).
    @param  addr
            The address to send the telegram to (unicast).
    @param  groups
            Output parameter returning the groups mask of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameter is set.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readmult(uint16_t addr, uint16_t *groups ) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readmult(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readmult(&resp, groups);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readmult(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" groups=0x%04X\n", *groups );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 0D SETMULT


static aoresult_t aoosp_con_setmult(aoosp_tele_t * tele, uint16_t addr, uint16_t groups) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;
  if( groups & ~0x7FFF        ) return aoresult_osp_arg;

  // Set constants
  const uint8_t payloadsize = 2;
  const uint8_t tid = 0x0D; // SETMULT
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = BITS_SLICE(groups,8,16);
  tele->data[4] = BITS_SLICE(groups,0,8);

  tele->data[5] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a SETMULT telegram.
            Assigns the addressed node to zero or more of the 15 groups.
    @param  addr
            The address to send the telegram to (unicast),
            (theoretically, use 0 for broadcast, or 3F0..3FE for a group).
    @param  groups
            The LSB 15 bits indicate if the node is assigned to that group.
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_setmult(uint16_t addr, uint16_t groups) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_setmult(&tele, addr, groups);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("setmult(0x%03X,0x%02X)",addr,groups);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 0E -- free


// ==========================================================================
// Telegram 0F SYNC


static aoresult_t aoosp_con_sync(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x0F; // SYNC
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a SYNC telegram.
            A sync event (via external pin or via this command), activates
            all drivers with pre-configured settings.
    @param  addr
            The address to send the telegram to (unicast).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_sync(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_sync(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("sync(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  };
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 10 -- free


// ==========================================================================
// Telegram 11 IDLE


static aoresult_t aoosp_con_idle(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x11; // IDLE
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a IDLE telegram, deselecting the active area of the OTP.
    @param  addr
            The address to send the telegram to (unicast).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   Part of OTP management telegrams (IDLE, FOUNDRY, CUST, BURN,
            AREAD, LOAD, GLOAD, and SETOTP). These requires test mode,
            which is activated by sending the correct password (SETTESTPW).
    @note   The OTP controller can be in three modes: idle, foundry or
            customer. In idle no OTP area is active, in foundry mode the
            foundry area is active and in customer mode the customer area
            is active. It is necessary to activate an area when issuing a
            BURN or LOAD command. After the BURN or LOAD command, it is
            advised to select the idle mode again by sending this telegram.
*/
aoresult_t aoosp_send_idle(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_idle(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("idle(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 12 FOUNDRY


static aoresult_t aoosp_con_foundry(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x12; // FOUNDRY
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a FOUNDRY telegram, activating the foundry area in the OTP.
    @param  addr
            The address to send the telegram to (unicast).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   Part of OTP management, see aoosp_send_idle().
*/
aoresult_t aoosp_send_foundry(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_foundry(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("foundry(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 13 CUST


static aoresult_t aoosp_con_cust(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x13; // CUST
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a CUST telegram, activating the customer area in the OTP.
    @param  addr
            The address to send the telegram to (unicast).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   Part of OTP management, see aoosp_send_idle().
*/
aoresult_t aoosp_send_cust(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_cust(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("cust(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 14 BURN


static aoresult_t aoosp_con_burn(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x14; // BURN
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a BURN telegram. Burns the fuses of the OTP (copying its
            RAM mirror to fuses) for the active area.
    @param  addr
            The address to send the telegram to (unicast).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   Part of OTP management, see aoosp_send_idle().
    @note   Recall that aoosp_send_setotp() only writes to the OTP mirror.
            The BURN command actually copies the mirror to the OTP, making
            the contents persistent over power-cycles.
    @note   Note that OTP fuses can only be burned from 0 to 1, not the
            other way round.
    @note   SAID must be in test mode (SETTESTPW), RAM mirror must have
            desired contents, Cust_lock bit in OTP (1E.7) shall not be set,
            the customer area shall be active (by sending CUST), and the SAID
            supply voltage shall be lowered (2.7V). Only with these
            conditions the BURN command will succeed.
    @note   After the BURN command, switch to IDLE, and leave test mode
            (SETTESTPW with e.g. 0).
    @note   One could verify success of burning, by doing a LOAD (from OTP
            to RAM mirror) and checking the RAM contents.
    @note   Seq: SETTESTPW(x), CUST, lower V, BURN, IDLE, SETTESPW(0), test with LOAD.
*/
aoresult_t aoosp_send_burn(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_burn(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("burn(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 15 AREAD (part of OTP management, but for internal tests)


static aoresult_t aoosp_con_aread(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x15; // AREAD
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a AREAD telegram.
    @param  addr
            The address to send the telegram to (unicast).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   Part of OTP management, see aoosp_send_idle().
    @note   This is an internal test telegram, performing analogue reads
            on the OTP fuses, the result is communicated via chip pins.
            Typically executed after a BURN.
*/
aoresult_t aoosp_send_aread(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_aread(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("aread(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 16 LOAD


static aoresult_t aoosp_con_load(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x16; // LOAD
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a LOAD telegram. Copies the OTP fuses to the OTP mirror RAM.
    @param  addr
            The address to send the telegram to (unicast).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   Part of OTP management, see aoosp_send_idle().
    @note   There is an implicit LOAD at SAID startup. Giving an explicit
            LOAD command makes sense after a BURN, to check if the
            new contents are successfully burned.
    @note   When using LOAD to check if BURN was successful, it is suggested
            to send a GLOAD before LOAD; this configures the LOAD command to
            be more strict (sets different guard-band for LOAD operation),
            checking if the fuses are burned thoroughly (robust in time and
            over temperature).
    @note   Before LOAD make sure an area is selected (e.g. by sending CUST).
            After LOAD command, switch to back IDLE, and leave test mode
            (SETTESTPW with e.g. 0).
    @note   Seq: SETTESTPW(x), CUST, [GLOAD,] LOAD, IDLE, SETTESPW(0), check mirror.
*/
aoresult_t aoosp_send_load(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_load(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("load(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 17 GLOAD


static aoresult_t aoosp_con_gload(aoosp_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x17; // GLOAD
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a GLOAD telegram. This sets a different guard-band for the
            subsequent LOAD operation, checking if the fuses are burned
            thoroughly.
    @param  addr
            The address to send the telegram to (unicast).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   Part of OTP management, see aoosp_send_idle().
    @note   This only configures the guard-band for loading, the actual
            loading starts by sending LOAD.
    @note   Before GLOAD make sure an area is selected (e.g. by sending CUST).
            After LOAD issue an IDLE to reset the strict guard-band.

*/
aoresult_t aoosp_send_gload(uint16_t addr) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_gload(&tele,addr);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("gload(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n");
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 18 I2CREAD


static aoresult_t aoosp_con_i2cread8(aoosp_tele_t * tele, uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t count) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;
  if( daddr7>127             ) return aoresult_osp_arg;
//if( raddr>255              ) return aoresult_osp_arg;
  if( count<1 || count>8     ) return aoresult_osp_arg;

  // Set constants
  const uint8_t payloadsize = 3;
  const uint8_t tid = 0x18; // I2CREAD
  // if( respsize ) *respsize = 4+0; // nothing

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = daddr7 << 1; // 7 bits address needs shifting
  tele->data[4] = raddr;
  tele->data[5] = count;

  tele->data[6] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a I2CREAD telegram.
            This requests a SAID to master a read on its I2C bus.
    @param  addr
            The address to send the telegram to (unicast),
            (theoretically use 0 for broadcast, or 3F0..3FE for group).
    @param  daddr7
            The 7 bits I2C device address used in mastering the read.
    @param  raddr
            The 8 bits register address used in mastering the read.
    @param  count
            The number of bytes to read from the I2C device (1..8).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   After I2CREAD, use READI2CCFG to check if the I2C transaction
            was successful. Then use READLAST to get the bytes the SAID read
            from the I2C device.
    @note   The SAID must have the I2C enable bit set in its OTP.
            On startup, send SETCURCHN to power the I2C bus.
    @note   The current implementation only supports the 8 bit mode.
    @note   The I2C transaction takes time, so wait after sending this telegram.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_i2cread8(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t count ) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_i2cread8(&tele,addr,daddr7,raddr,count);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("i2cread(0x%03X,0x%02X,0x%02X,%d)",addr,daddr7,raddr,count );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 19 I2CWRITE


static aoresult_t aoosp_con_i2cwrite8(aoosp_tele_t * tele, uint16_t addr, uint8_t daddr7, uint8_t raddr, const uint8_t * buf, int count) {
  // Check input parameters
  if( tele==0 || buf==0        ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr)   ) return aoresult_osp_addr;
  if( daddr7>127               ) return aoresult_osp_arg;
//if( raddr>255                ) return aoresult_osp_arg;
  if( count<1                  ) return aoresult_osp_arg; // SAID wants minimally one I2C byte
  if( count+2>8                ) return aoresult_osp_arg; // telegram payload cannot exceed 8 bytes (two byte is for daddr/raddr)
  if( count+2==5 || count+2==7 ) return aoresult_osp_arg; // telegram payloads 5 and 7 are not supported in OSP

  // Set constants
  const uint8_t payloadsize = 2+count; // daddr, raddr and buf size (count)
  const uint8_t tid = 0x19; // I2CWRITE
  // if( respsize ) *respsize = 4+0; // nothing

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = daddr7 << 1; // 7 bits device address needs shifting
  tele->data[4] = raddr;
  for( int i=0; i<count; i++ ) tele->data[5+i] = buf[i];

  tele->data[3+2+count] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a I2CWRITE telegram.
            This requests a SAID to master a write on its I2C bus.
    @param  addr
            The address to send the telegram to (unicast),
            (theoretically, use 0 for broadcast, or 3F0..3FE for group).
    @param  daddr7
            The 7 bits I2C device address used in mastering the write.
    @param  raddr
            The 8 bits register address used in mastering the write.
    @param  buf
            Pointer to buffer containing the bytes to send to the I2C device.
    @param  count
            The number of bytes to write from the buffer (1, 2, 4, or 6).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   After I2CWRITE, use READI2CCFG to check if the I2C transaction was successful.
    @note   The SAID must have the I2C enable bit set in its OTP.
            On startup, send SETCURCHN to power the I2C bus.
    @note   The current implementation only supports the 8 bit mode.
    @note   The I2C transaction takes time, so wait after sending this telegram.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_i2cwrite8(uint16_t addr, uint8_t daddr7, uint8_t raddr, const uint8_t * buf, int count) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_i2cwrite8(&tele,addr,daddr7,raddr,buf,count);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("i2cwrite(0x%03X,0x%02X,0x%02X,0x[%s])",addr,daddr7,raddr,aoosp_prt_bytes(buf,count));
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 1A -- free
// Telegram 1B -- free
// Telegram 1C -- free
// Telegram 1D -- free


// ==========================================================================
// Telegram 1E READLAST


static aoresult_t aoosp_con_readlast(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x1E; // READLAST
  if( respsize ) *respsize = 4+8; // i2c read buffer

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readlast(aoosp_tele_t * tele, uint8_t * buf, int size) {
  // Set constants
  const uint8_t payloadsize = 8;
  // Check telegram consistency
  if( tele==0 || buf==0                        ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x1E      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;
  if( size<1 || size>8                         ) return aoresult_osp_arg;

  // Get fields
  for( int i=0; i<size; i++ ) buf[i] = tele->data[11-size+i];

  return aoresult_ok;
}


/*!
    @brief  Sends a READLAST telegram and receives its response.
            This requests a SAID to return the result of the last I2CREAD.
    @param  addr
            The address to send the telegram to (unicast).
    @param  buf
            Pointer to buffer to hold the retrieved bytes.
    @param  size
            The number of bytes to retrieve
            (the buffer should have at least that size).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   First send a I2CREAD to get bytes from an I2C device into the SAID.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   The response telegram always has a size of 8 irrespective
            of how many bytes were read with I2CREAD. If less bytes were
            read in I2CREAD, they are padded. Typically a call to
            `aoosp_send_i2cread8(addr,daddr7,raddr,n)` is followed by a call
            to `aoosp_send_readlast(addr,buf,n)` with the same `addr` and
            same `n`.
*/
aoresult_t aoosp_send_readlast(uint16_t addr, uint8_t * buf, int size) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readlast(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readlast(&resp, buf, size);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readlast(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" i2c %s\n", aoosp_prt_bytes(buf,size) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 1F -- free
// Telegram 20 -- RESET has no SR
// Telegram 21 CLRERROR_SR
// Telegram 22 -- INIBIDIR has no SR
// Telegram 23 -- INITLOOP has no SR
// Telegram 24 GOSLEEP_SR


// ==========================================================================
// Telegram 25 GOACTIVE_SR


static aoresult_t aoosp_con_goactive_sr(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x25; // GOACTIVE_SR
  if( respsize ) *respsize = 4+2; // temp,stat

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_goactive_sr(aoosp_tele_t * tele, uint8_t * temp, uint8_t * stat) {
  // Set constants
  const uint8_t payloadsize = 2;
  // Check telegram consistency
  if( tele==0 || stat==0                       ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x25      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *temp = tele->data[3];
  *stat = tele->data[4];

  return aoresult_ok;
}


/*!
    @brief  Sends an GOACTIVE_SR telegram and receives a status response.
            This switches the state of the addressed node to active
            (allowing to switch on LEDs).
    @param  addr
            The address to send the telegram to (unicast).
    @param  temp
            Output parameter returning the raw temperature of the addressed node.
    @param  stat
            Output parameter returning the (system) status of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameters are set.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_goactive_sr(uint16_t addr, uint8_t * temp, uint8_t * stat) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_goactive_sr(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_goactive_sr(&resp, temp, stat);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("goactive_sr(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" temp=0x%02X=%d stat=0x%02X=%s", *temp, aoosp_prt_temp_said(*temp), *stat, aoosp_prt_stat_said(*stat) );
    Serial.printf(" (%d, %s)\n",  aoosp_prt_temp_rgbi(*temp), aoosp_prt_stat_rgbi(*stat) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 26 GODEEPSLEEP_SR
// Telegram 27 -- IDENTIFY has no SR
// Telegram 28 -- P4ERRBIDIR has no SR
// Telegram 29 -- P4ERRLOOP has no SR
// Telegram 2A -- ASKTINFO has no SR
// Telegram 2B -- ASKVINFO has no SR
// Telegram 2C -- READMULT has no SR
// Telegram 2D SETMULT_SR
// Telegram 2E -- free
// Telegram 2F -- SYNC has no SR
// Telegram 30 -- free
// Telegram 31 IDLE_SR
// Telegram 32 FOUNDRY_SR
// Telegram 33 CUST_SR
// Telegram 34 BURN_SR
// Telegram 35 AREAD_SR
// Telegram 36 LOAD_SR
// Telegram 37 GLOAD_SR
// Telegram 38 I2CREAD_SR
// Telegram 39 I2CWRITE_SR


// ==========================================================================
// Telegram 40 READSTAT


static aoresult_t aoosp_con_readstat(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x40; // READSTAT
  if( respsize ) *respsize = 4+1; //  stat

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readstat(aoosp_tele_t * tele, uint8_t * stat) {
  // Set constants
  const uint8_t payloadsize = 1;
  // Check telegram consistency
  if( tele==0 || stat==0                       ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x40      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *stat = tele->data[3];

  return aoresult_ok;
}


/*!
    @brief  Sends a READSTAT telegram and receives its response.
            Asks the addressed node to respond with its (system) status.
    @param  addr
            The address to send the telegram to (unicast).
    @param  stat
            Output parameter returning the (system) status of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameter is set.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readstat(uint16_t addr, uint8_t * stat) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readstat(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readstat(&resp, stat);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readstat(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" stat=0x%02X=%s", *stat, aoosp_prt_stat_said(*stat) );
    Serial.printf(" (%s)\n", aoosp_prt_stat_rgbi(*stat) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 41 -- no SETSTAT


// ==========================================================================
// Telegram 42 READTEMPSTAT


static aoresult_t aoosp_con_readtempstat(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x42; // READTEMPSTAT
  if( respsize ) *respsize = 4+2; // temp, stat

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readtempstat(aoosp_tele_t * tele, uint8_t * temp, uint8_t * stat) {
  // Set constants
  const uint8_t payloadsize = 2;
  // Check telegram consistency
  if( tele==0 || temp==0 || stat==0            ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x42      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *temp = tele->data[3];
  *stat = tele->data[4];

  return aoresult_ok;
}


/*!
    @brief  Sends a READTEMPSTAT telegram and receives its response.
            Asks the addressed node to respond with its temperature
            and (system) status.
    @param  addr
            The address to send the telegram to (unicast).
    @param  temp
            Output parameter returning the raw temperature of the addressed node.
    @param  stat
            Output parameter returning the (system) status of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameters are set.
    @note   Converting raw temperature to Celsius depends on the node type.
            See e.g. `aoosp_prt_temp_rgbi()` or `aoosp_prt_temp_said()`.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readtempstat(uint16_t addr, uint8_t * temp, uint8_t * stat) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readtempstat(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readtempstat(&resp, temp, stat);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readtempstat(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" temp=0x%02X=%d stat=0x%02X=%s", *temp, aoosp_prt_temp_said(*temp), *stat, aoosp_prt_stat_said(*stat) );
    Serial.printf(" (%d, %s)\n",  aoosp_prt_temp_rgbi(*temp), aoosp_prt_stat_rgbi(*stat) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 43 -- no SETTEMPSTAT


// ==========================================================================
// Telegram 44 READCOMST


static aoresult_t aoosp_con_readcomst(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x44; // READCOMST
  if( respsize ) *respsize = 4+1; // comst

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readcomst(aoosp_tele_t * tele, uint8_t * com) {
  // Set constants
  const uint8_t payloadsize = 1;
  // Check telegram consistency
  if( tele==0 || com==0                        ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x44      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *com= tele->data[3];

  return aoresult_ok;
}


/*!
    @brief  Sends a READCOMST telegram and receives its response.
            Asks the addressed node to respond with its communication status
            (how its SIO ports are configured 00=LVDS, 01=EOL, 10=MCU, 11=CAN).
    @param  addr
            The address to send the telegram to (unicast).
    @param  com
            Output parameter returning the communication status of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameter is set.
    @note   Status fields depend on the node type.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readcomst(uint16_t addr, uint8_t * com) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readcomst(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readcomst(&resp, com);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readcomst(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" com=0x%02X=%s", *com, aoosp_prt_com_said(*com) );
    Serial.printf(" (%s)\n", aoosp_prt_com_rgbi(*com) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 45 -- no SETCOMST


// ==========================================================================
// Telegram 46 READLEDST (RGBi only)


static aoresult_t aoosp_con_readledst(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x46; // READLEDST
  if( respsize ) *respsize = 4+1; // ledst

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readledst(aoosp_tele_t * tele, uint8_t * ledst) {
  // Set constants
  const uint8_t payloadsize = 1;
  // Check telegram consistency
  if( tele==0 || ledst==0                      ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x46      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *ledst = tele->data[3];

  return aoresult_ok;
}


/*!
    @brief  Sends a READLEDST telegram and receives its response.
            Asks the addressed node to respond with its LED status;
            whether the R, G, or B is "open" or "shorted".
    @param  addr
            The address to send the telegram to (unicast).
    @param  ledst
            Output parameter returning the LED status of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameter is set.
    @note   A node must be in state "active" in order for the LEDs to be driven.
    @note   "Open" means the driver output pin has a low voltage:
            either the output pin is dangling (not connected at all),
            or the pin is grounded (or the LED is blown open).
    @note   "Shorted" means the driver output pin has a high voltage: the
            output pin is connected to VCC (or the LED is short circuited).
    @note   Although this telegram ID (46) is the same as for READLEDSTCHN, the
            payload is specific for single channel PWM devices like RGBi's.
            For multi channel PWM devices, like SAID, use READLEDSTCHN.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readledst(uint16_t addr, uint8_t * ledst) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readledst(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readledst(&resp, ledst);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readledst(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" ledst=0x%02X=%s\n", *ledst, aoosp_prt_ledst(*ledst) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 46 READLEDSTCHN (SAID only)


static aoresult_t aoosp_con_readledstchn(aoosp_tele_t * tele, uint16_t addr, uint8_t chn, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                    ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr)     ) return aoresult_osp_addr;
  if( chn!=0 && chn!=1 && chn!=2 ) return aoresult_osp_arg;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x46; // READLEDST
  if( respsize ) *respsize = 4+1; // ledst

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = chn;

  tele->data[4] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readledstchn(aoosp_tele_t * tele, uint8_t * ledst) {
  // Set constants
  const uint8_t payloadsize = 1;
  // Check telegram consistency
  if( tele==0 || ledst==0                      ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x46      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *ledst = tele->data[3];

  return aoresult_ok;
}


/*!
    @brief  Sends a READLEDSTCHN telegram and receives its response.
            Asks the addressed node to respond with its LED status on
            channel `chn`; whether the R, G, or B is "open" (not connected)
            or "short" (connected to +5V).
    @param  addr
            The address to send the telegram to (unicast).
    @param  chn
            The channel of the node for which the LED status is requested.
    @param  ledst
            Output parameter returning the LED status of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameter is set.
    @note   See the notes for `aoosp_send_readledst`.
    @note   Although this telegram ID (46) is the same as for READLEDST, the
            payload is specific for multi channel PWM devices, like SAID.
            For single channel PWM devices like RGBi's, use READLEDST.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readledstchn(uint16_t addr, uint8_t chn, uint8_t * ledst) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readledstchn(&tele,addr,chn,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readledstchn(&resp, ledst);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readledstchn(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" ledst=0x%02X=%s\n", *ledst, aoosp_prt_ledst(*ledst) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 47 -- no SETLEDST


// ==========================================================================
// Telegram 48 READTEMP


static aoresult_t aoosp_con_readtemp(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x48; // READTEMP
  if( respsize ) *respsize = 4+1; // temp

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readtemp(aoosp_tele_t * tele, uint8_t * temp) {
  // Set constants
  const uint8_t payloadsize = 1;
  // Check telegram consistency
  if( tele==0 || temp==0                       ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x48      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *temp = tele->data[3];

  return aoresult_ok;
}


/*!
    @brief  Sends a READTEMP telegram and receives its response.
            Asks the addressed node to respond with its raw temperature.
    @param  addr
            The address to send the telegram to (unicast).
    @param  temp
            Output parameter returning the raw temperature of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameter is set.
    @note   Converting raw temperature to Celsius depends on the node type.
            See e.g. `aoosp_prt_temp_rgbi()` or `aoosp_prt_temp_said()`.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readtemp(uint16_t addr, uint8_t * temp) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readtemp(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readtemp(&resp, temp);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readtemp(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" temp=0x%02X=%d", *temp, aoosp_prt_temp_said(*temp) );
    Serial.printf(" (%d)\n", aoosp_prt_temp_rgbi(*temp) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 49 -- no SETTEMP
// Telegram 4A READOTTH
// Telegram 4B SETOTTH


// ==========================================================================
// Telegram 4C READSETUP


static aoresult_t aoosp_con_readsetup(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x4C; // READSETUP
  if( respsize ) *respsize = 4+1; // flags

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readsetup(aoosp_tele_t * tele, uint8_t *flags ) {
  // Set constants
  const uint8_t payloadsize = 1;
  // Check telegram consistency
  if( tele==0 || flags==0                      ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x4C      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *flags= tele->data[3];

  return aoresult_ok;
}


/*!
    @brief  Sends a READSETUP telegram and receives its response.
            Asks the addressed node to respond with its setup (e.g. CRC check enabled).
    @param  addr
            The address to send the telegram to (unicast).
    @param  flags
            Output parameter returning the setup of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameter is set.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readsetup(uint16_t addr, uint8_t *flags ) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readsetup(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readsetup(&resp, flags);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readsetup(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" flags=0x%02X=%s\n", *flags, aoosp_prt_setup(*flags) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 4D SETSETUP


static aoresult_t aoosp_con_setsetup(aoosp_tele_t * tele, uint16_t addr, uint8_t flags) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x4D; // SETSETUP
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = flags;

  tele->data[4] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a SETSETUP telegram.
            Sets the setup of the addressed node (e.g. CRC check enabled).
    @param  addr
            The address to send the telegram to (unicast),
            (use 0 for broadcast, or 3F0..3FE for group).
    @param  flags
            The new setup of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_setsetup(uint16_t addr, uint8_t flags) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_setsetup(&tele, addr, flags);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("setsetup(0x%03X,0x%02X)",addr,flags);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 4E READPWM (RGBi only)


static aoresult_t aoosp_con_readpwm(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x4E; // READPWM
  if( respsize ) *respsize = 4+6; // red, green, blue

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


// the three 1-bit daytimes flags are clubbed into one daytimes argument
static aoresult_t aoosp_des_readpwm(aoosp_tele_t * tele, uint16_t *red, uint16_t *green, uint16_t *blue, uint8_t *daytimes) {
  // Set constants
  const uint8_t payloadsize = 6;
  // Check telegram consistency
  if( tele==0 || red==0 || green==0 || blue==0 || daytimes==0 ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                               ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)                    ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA                      ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x4E                     ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0                     ) return aoresult_osp_crc;

  // Get fields
  *red  = BITS_SLICE(tele->data[3],0,7)<<8 | tele->data[4] ;
  *green= BITS_SLICE(tele->data[5],0,7)<<8 | tele->data[6] ;
  *blue = BITS_SLICE(tele->data[7],0,7)<<8 | tele->data[8] ;
  *daytimes = BITS_SLICE(tele->data[3],7,8)<<2 | BITS_SLICE(tele->data[5],7,8)<<1 |  BITS_SLICE(tele->data[7],7,8);

  return aoresult_ok;
}


/*!
    @brief  Sends a READPWM telegram and receives its response.
            Asks the addressed node to respond with its PWM settings
            (for single channel nodes).
    @param  addr
            The address to send the telegram to (unicast).
    @param  red
            Output parameter returning the PWM setting for red.
    @param  green
            Output parameter returning the PWM setting for green.
    @param  blue
            Output parameter returning the PWM setting for blue.
    @param  daytimes
            Output parameter returning the daytime flags
            (bit 2 indicates daytime for red, bit 1 for green, 0 for blue).
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameters are set.
    @note   Although this telegram ID (4E) is the same as for READPWMCHN, the
            payload is specific for single channel PWM devices like RGBi's.
            For multi channel PWM devices, like SAID, use READPWMCHN.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readpwm(uint16_t addr, uint16_t *red, uint16_t *green, uint16_t *blue, uint8_t *daytimes) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readpwm(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readpwm(&resp, red, green, blue, daytimes);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readpwm(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" rgb=%s\n", aoosp_prt_pwm_rgbi(*red,*green,*blue,*daytimes) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 4E READPWMCHN (SAID only)


static aoresult_t aoosp_con_readpwmchn(aoosp_tele_t * tele, uint16_t addr, uint8_t chn, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                    ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr)     ) return aoresult_osp_addr;
  if( chn!=0 && chn!=1 && chn!=2 ) return aoresult_osp_arg;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x4E; // READPWMCHN
  if( respsize ) *respsize = 4+6; // red, green, blue

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = chn;

  tele->data[4] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


// the meaning of the 16 color bits varies, not detailed here at telegram level
static aoresult_t aoosp_des_readpwmchn(aoosp_tele_t * tele, uint16_t *red, uint16_t *green, uint16_t *blue ) {
  // Set constants
  const uint8_t payloadsize = 6;
  // Check telegram consistency
  if( tele==0 || red==0 || green==0 || blue==0 ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x4E      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *red  = tele->data[3]<<8 | tele->data[4] ;
  *green= tele->data[5]<<8 | tele->data[6] ;
  *blue = tele->data[7]<<8 | tele->data[8] ;

  return aoresult_ok;
}


/*!
    @brief  Sends a READPWMCHN telegram and receives its response.
            Asks the addressed node to respond with its PWM settings
            of one of its channel.
    @param  addr
            The address to send the telegram to (unicast).
    @param  chn
            The channel of the node for which the PWM settings are requested.
    @param  red
            Output parameter returning the PWM setting for red.
    @param  green
            Output parameter returning the PWM setting for green.
    @param  blue
            Output parameter returning the PWM setting for blue.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameters are set.
    @note   Although this telegram ID (4E) is the same as for READPWM, the
            payload is specific for multi channel PWM devices like SAIDs.
            For single channel PWM devices, like RGBi, use READPWM.
    @note   The meaning of the 16 bits varies, they are not detailed
            here at telegram level.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readpwmchn(uint16_t addr, uint8_t chn, uint16_t *red, uint16_t *green, uint16_t *blue ) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readpwmchn(&tele,addr,chn,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readpwmchn(&resp, red, green, blue);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readpwmchn(0x%03X,%X)",addr,chn);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" rgb=%s\n", aoosp_prt_pwm_said(*red,*green,*blue) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 4F SETPWM (RGBi only)


static aoresult_t aoosp_con_setpwm(aoosp_tele_t * tele, uint16_t addr, uint16_t red, uint16_t green, uint16_t blue, uint8_t daytimes) {
  // Check input parameters
  if( tele==0                   ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr)    ) return aoresult_osp_addr;
  if( red      & ~BITS_MASK(15) ) return aoresult_osp_arg;
  if( green    & ~BITS_MASK(15) ) return aoresult_osp_arg;
  if( blue     & ~BITS_MASK(15) ) return aoresult_osp_arg;
  if( daytimes & ~BITS_MASK(3)  ) return aoresult_osp_arg;

  // Set constants
  const uint8_t payloadsize = 6;
  const uint8_t tid = 0x4F; // SETPWM
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = BITS_SLICE(daytimes,2,3)<<7 | BITS_SLICE(red,8,15);
  tele->data[4] = BITS_SLICE(red,0,8);
  tele->data[5] = BITS_SLICE(daytimes,1,2)<<7 | BITS_SLICE(green,8,15);
  tele->data[6] = BITS_SLICE(green,0,8);
  tele->data[7] = BITS_SLICE(daytimes,0,1)<<7 | BITS_SLICE(blue,8,15);
  tele->data[8] = BITS_SLICE(blue,0,8);

  tele->data[9] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a SETPWM telegram.
            Configures the PWM settings of the addressed node
            (for single channel nodes).
    @param  addr
            The address to send the telegram to (unicast),
            (use 0 for broadcast, or 3F0..3FE for group).
    @param  red
            The PWM setting for red (15 bit).
    @param  green
            The PWM setting for green (15 bit).
    @param  blue
            The PWM setting for blue (15 bit).
    @param  daytimes
            The daytime flags (3 bit):
            bit 2 indicates daytime for red, bit 1 for green, 0 for blue.
    @return aoresult_ok if all ok, otherwise an error code.
    @note   A node must be active in order for the PWMs to switch on.
    @note   Although this telegram ID (4F) is the same as for SETPWMCHN, the
            payload is specific for single channel PWM devices like RGBi's.
            For multi channel PWM devices, like SAID, use SETPWMCHN.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_setpwm(uint16_t addr, uint16_t red, uint16_t green, uint16_t blue, uint8_t daytimes) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_setpwm(&tele, addr, red, green, blue, daytimes);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("setpwm(0x%03X,0x%04X,0x%04X,0x%04X,%X)",addr,red,green,blue,daytimes);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 4F SETPWMCHN (SAID only)


static aoresult_t aoosp_con_setpwmchn(aoosp_tele_t * tele, uint16_t addr, uint8_t chn, uint16_t red, uint16_t green, uint16_t blue) {
  // Check input parameters
  if( tele==0                    ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr)     ) return aoresult_osp_addr;
  if( chn!=0 && chn!=1 && chn!=2 ) return aoresult_osp_arg;

  // Set constants
  const uint8_t payloadsize = 8;
  const uint8_t tid = 0x4F; // SETPWMCHN (same SETPWM of OSP V1)
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = chn;
  tele->data[4] = 0xFF; // dummy;

  tele->data[5] = BITS_SLICE(red,8,16);
  tele->data[6] = BITS_SLICE(red,0,8);
  tele->data[7] = BITS_SLICE(green,8,16);
  tele->data[8] = BITS_SLICE(green,0,8);
  tele->data[9] = BITS_SLICE(blue,8,16);
  tele->data[10]= BITS_SLICE(blue,0,8);

  tele->data[11] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a SETPWMCHN telegram.
            Configures the PWM settings of one channel of the addressed node.
    @param  addr
            The address to send the telegram to (unicast),
            (use 0 for broadcast, or 3F0..3FE for group).
    @param  chn
            The channel of the node for which the PWM settings are configured.
    @param  red
            The PWM setting for red (16 bit).
    @param  green
            The PWM setting for green (16 bit).
    @param  blue
            The PWM setting for blue (16 bit).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   A node must be active in order for the PWMs to switch on.
    @note   Although this telegram ID (4F) is the same as for SETPWM, the
            payload is specific for multi channel PWM devices like SAIDs.
            For single channel PWM devices, like RGBi, use SETPWM.
    @note   The meaning of the 16 bits varies, they are not detailed
            here at telegram level. For SAID the 15 MSB bits form the PWM
            value, and the LSB bit en/disables dithering (this may be
            regarded as bit 16 of PWM value).
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_setpwmchn(uint16_t addr, uint8_t chn, uint16_t red, uint16_t green, uint16_t blue) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_setpwmchn(&tele, addr, chn, red, green, blue);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("setpwmchn(0x%03X,%X,0x%04X,0x%04X,0x%04X)",addr,chn,red,green,blue);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 50 READCURCHN


static aoresult_t aoosp_con_readcurchn(aoosp_tele_t * tele, uint16_t addr, uint8_t chn, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                    ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr)     ) return aoresult_osp_addr;
  if( chn!=0 && chn!=1 && chn!=2 ) return aoresult_osp_arg;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x50; // READCURCHN
  if( respsize ) *respsize = 4+2; // 3*4 PWM bits, 4 flags

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = chn;

  tele->data[4] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readcurchn(aoosp_tele_t * tele, uint8_t *flags, uint8_t *rcur, uint8_t *gcur, uint8_t *bcur ) {
  // Set constants
  const uint8_t payloadsize = 2;
  // Check telegram consistency
  if( tele==0 || flags==0 || rcur==0 || gcur==0 || bcur==0 ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                            ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)                 ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA                   ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x50                  ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0                  ) return aoresult_osp_crc;

  // Get fields
  *flags= BITS_SLICE(tele->data[3],4,8);
  *rcur  = BITS_SLICE(tele->data[3],0,4);
  *gcur= BITS_SLICE(tele->data[4],4,8);
  *bcur = BITS_SLICE(tele->data[4],0,4);

  return aoresult_ok;
}


/*!
    @brief  Sends a READCURCHN telegram and receives its response.
            Asks the addressed node to respond with the current
            levels of the specified channel.
    @param  addr
            The address to send the telegram to (unicast).
    @param  chn
            The channel of the node for which the current levels are requested.
    @param  flags
            Output parameter returning the current flags of the addressed node and channel.
    @param  rcur
            Output parameter returning the current level for red of the addressed node and channel.
    @param  gcur
            Output parameter returning the current level for green of the addressed node and channel.
    @param  bcur
            Output parameter returning the current level for blue of the addressed node and channel.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameters are set.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readcurchn(uint16_t addr, uint8_t chn, uint8_t *flags, uint8_t *rcur, uint8_t *gcur, uint8_t *bcur ) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readcurchn(&tele,addr,chn,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readcurchn(&resp, flags, rcur, gcur, bcur);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readcurchn(0x%03X,%X)",addr,chn);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" flags=%s rcur=%X gcur=%X bcur=%X\n", aoosp_prt_curchn(*flags), *rcur,*gcur,*bcur );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 51 SETCURCHN


#define AOOSP_CUR_NORM_OK(v) ( 0b0000<=(v) && (v)<=0b0100 )
#define AOOSP_CUR_AGE_OK(v)  ( 0b1000<=(v) && (v)<=0b1011 )
#define AOOSP_CUR_OK(v)      ( AOOSP_CUR_NORM_OK(v) || AOOSP_CUR_AGE_OK(v) )
static aoresult_t aoosp_con_setcurchn(aoosp_tele_t * tele, uint16_t addr, uint8_t chn, uint8_t flags, uint8_t rcur, uint8_t gcur, uint8_t bcur ) {
  // Check input parameters
  if( tele==0                    ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr)     ) return aoresult_osp_addr;
  if( flags  & ~0x07             ) return aoresult_osp_arg;
  if( !AOOSP_CUR_OK(rcur)        ) return aoresult_osp_arg;
  if( !AOOSP_CUR_OK(gcur)        ) return aoresult_osp_arg;
  if( !AOOSP_CUR_OK(bcur)        ) return aoresult_osp_arg;
  if( chn!=0 && chn!=1 && chn!=2 ) return aoresult_osp_arg;

  // Set constants
  const uint8_t payloadsize = 3;
  const uint8_t tid = 0x51; // SETCURCHN
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = chn;

  tele->data[4] = flags<<4 | rcur;
  tele->data[5] = gcur<<4 | bcur ;

  tele->data[6] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a SETCURCHN telegram.
            Configures the current levels of the addressed node for the
            specified channel.
    @param  addr
            The address to send the telegram to (unicast),
            (use 0 for broadcast, or 3F0..3FE for group).
    @param  chn
            The channel of the node for which the current levels are requested.
    @param  flags
            The 4 bit current flags.
    @param  rcur
            The 4 bit current level for red.
    @param  gcur
            The 4 bite current level for green.
    @param  bcur
            The 4 bit current level for blue.
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_setcurchn(uint16_t addr, uint8_t chn, uint8_t flags, uint8_t rcur, uint8_t gcur, uint8_t bcur ) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_setcurchn(&tele, addr, chn, flags, rcur, gcur, bcur);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("setcurchn(0x%03X,%X,%s,%X,%X,%X)",addr,chn,aoosp_prt_curchn(flags),rcur,gcur,bcur);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 52 READTCOEFF
// Telegram 53 SETTCOEFF


// ==========================================================================
// Telegram 54 READADC - asks the addressed node to respond with the ADC configuration (the data is in telegram 5C).


static aoresult_t aoosp_con_readadc(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x54; // READADC
  if( respsize ) *respsize = 4+1; // flags

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readadc(aoosp_tele_t * tele, uint8_t *flags ) {
  // Set constants
  const uint8_t payloadsize = 1;
  // Check telegram consistency
  if( tele==0 || flags==0                      ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x54      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *flags= tele->data[3];

  return aoresult_ok;
}


/*!
    @brief  Sends a READADC telegram and receives its response.
            Asks the addressed node to respond with its ADC
            configuration.
    @param  addr
            The address to send the telegram to (unicast).
    @param  flags
            Output parameter returning the ADC configuration
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameter is set.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   Use aoosp_send_readadcdat() to retrieve the ADC measurement data,
            instead of the ADC measurement configuration.
    @note   See aoosp_send_setadc() for details.
*/
aoresult_t aoosp_send_readadc(uint16_t addr, uint8_t *flags ) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readadc(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readadc(&resp, flags);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readadc(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" flags=0x%02X\n", *flags );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 55 SETADC  - configures the ADC to measure Vf of any of the drivers


static aoresult_t aoosp_con_setadc(aoosp_tele_t * tele, uint16_t addr, uint8_t flags) {
 // Check input parameters
 if( tele==0                ) return aoresult_outargnull;
 if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

 // Set constants
 const uint8_t payloadsize = 1;
 const uint8_t tid = 0x55; // SETADC
 //if( respsize ) *respsize = 4+0;

 // Build telegram
 tele->size    = payloadsize+4;

 tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
 tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
 tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

 tele->data[3] = flags;

 tele->data[4] = aoosp_crc( tele->data , tele->size - 1 );

 return aoresult_ok;
}


/*!
    @brief  Sends a SETADC telegram.
            Configures the ADC of the addressed node.
    @param  addr
            The address to send the telegram to (unicast),
            (use 0 for broadcast, or 3F0..3FE for group).
    @param  flags
            The new ADC configuration of the addressed node.
            Composed from AOOSP_ADC_FLAGS_xxx; typically one
            AOOSP_ADC_FLAGS_SYNC_xxx and one AOOSP_ADC_FLAGS_MUX_xxx.
    @return aoresult_ok if all ok, otherwise an error code.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   A SAID contains an ADC that constantly performs measurements. The
            datasheet calls this mode "no ADC measurement is requested", and
            technically this means that the ADC mux is in mux mode "auto"
            (flags=AOOSP_ADC_FLAGS_MUX_AUTO), which is the default. In mux
            mode auto these items are measured (by automatically changing the
            ADC mux in a round robin style)
              temperature sensor, Vf_R0, Vf_R1, Vf_R2,
              temperature sensor, Vf_B0, Vf_B1, Vf_B2, and
              temperature sensor, Vf_G0, Vf_G1, Vf_G2.
            The results of the temperature sensor measurement are internally
            stored, and available through telegrams READTEMP (or READTEMPSTAT)
            and ASKTINFO.
            The results of nine forward voltage measurements are also
            internally stored; all 9 are "maxed" together and stored in one
            register (VFMAX), which is available via telegram ASKVINFO.
            In mux mode auto the ADC has a typical cycle of 18ms. This means
            that in the auto cycle temperature is updated every 6ms.
    @note   If the ADC is configured with AOOSP_ADC_FLAGS_MUX_DRVxx (instead
            of AOOSP_ADC_FLAGS_MUX_AUTO), then the default ADC cycle is
            stopped. The datasheet calls this mode "an ADC measurement is
            requested", technically the ADC is in mux mode "driver" (aka
            "single driver" or "driver xx").
            In this mode, the device has a short cycle; it alternates between
            a temperature sensor measurement and a forward voltage measurement
            of driver "xx". In other words, temperature is still available,
            but ASKVINFO will give wrong results.
            In this mode, the reading from driver AOOSP_ADC_FLAGS_MUX_DRVxx
            is stored in ADCDAT, which is available via telegram READADCDAT.
            In mux mode driver the ADC has a cycle of 1.8ms. For the very
            first measurement after SETADC wait two cycles (because the
            switch of the mux to the new driver is somewhere in the middle
            of a cycle).
    @note   In addition to the two mux modes, the ADC has also has two sync
            modes: synced with the PWM engine (AOOSP_ADC_FLAGS_SYNC_ENA) or
            not (AOOSP_ADC_FLAGS_SYNC_DIS).
            The former configuration is typical when measuring the Vf of a
            LED (because that needs to be done when PWM is high), the latter
            configuration is typical when the ADC is used as a generic voltage
            measurement tool, with an externally applied voltage to the pad.
    @note   When the ADC is used to measure a forward voltage of an LED, the
            PWM high part must be at least 0x600 for the ADC to measure
            successfully.
    @note   When a driver pin is used as "general purpose" ADC input instead
            of a LED driving pin, the driver must be disabled: SETPWMCHN to 0.
    @note   The (state machine around the) ADC mux can lock up under specific
            circumstances. A locked mux will no longer switch to another input
            when instructed via a SETADC telegram. Also the cycle with the
            automatic measurements stops.
            The locked mux condition arises when the state machine is waiting
            for a PWM edge which is not coming. For example (1) a SETADC
            followed by SETPWMCHN to 0% can lock the mux (no PWM edge for
            synchronizing the ADC conversion), and (2) a SETADC followed by
            a SETADC with a different input can lock the mux.
            To unlock the mux, toggle the SYNC bit:
              AOOSP_ADC_FLAGS_SYNC_DIS then AOOSP_ADC_FLAGS_SYNC_ENA
    @note   The ADC measures voltage drops with respect to Vdd (5.0V).
            In other words, when this function reports 2.0V, the voltage drop
            from Vdd (over the LED typically) is 2.0V. This means that the
            LED has a Vf=2.0V (see left diagram).
            If the ADC is not connected to an LED, but used as a generic
            tool to measure the voltage at the pad, a 2V reported by this
            function means the pad has a voltage of 3V (see right diagram).

            Forward voltage of LED            Generic voltage measurement

            ------+-- 5.0V                    --------- 5.0V
                  |
                 LED  Vadc=2.0V => Vf=2.0V
                  |
            SAID--+                           SAID--o   Vadc=2.0V => Vpad=3.0V

*/
aoresult_t aoosp_send_setadc(uint16_t addr, uint8_t flags) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_setadc(&tele, addr, flags);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("setadc(0x%03X,0x%02X)",addr,flags);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 56 READI2CCFG (aka I2C status)


static aoresult_t aoosp_con_readi2ccfg(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x56; // READI2CCFG
  if( respsize ) *respsize = 4+1; // flags:speed

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readi2ccfg(aoosp_tele_t * tele, uint8_t *flags, uint8_t * speed ) {
  // Set constants
  const uint8_t payloadsize = 1;
  // Check telegram consistency
  if( tele==0 || flags==0 || speed==0          ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x56      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *flags = BITS_SLICE(tele->data[3],4,8);
  *speed = BITS_SLICE(tele->data[3],0,4);

  return aoresult_ok;
}


/*!
    @brief  Sends a READI2CCFG telegram and receives its response.
            Asks the addressed node to respond with the I2C configuration/status.
    @param  addr
            The address to send the telegram to (unicast).
    @param  flags
            Output parameter returning the I2C configuration flags of the addressed node.
    @param  speed
            Output parameter returning the I2C bus speed of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameters are set.
    @note   The I2C configuration register also double as I2C status register.
            For example twelve bit addressing and speed are configuration settings,
            whereas interrupt, ack/nack, and busy are status flags.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readi2ccfg(uint16_t addr, uint8_t *flags, uint8_t *speed ) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readi2ccfg(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readi2ccfg(&resp, flags, speed);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readi2ccfg(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" flags=0x%02X=%s speed=0x%02X=%d\n", *flags, aoosp_prt_i2ccfg(*flags), *speed, aoosp_prt_i2ccfg_speed(*speed) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 57 SETI2CCFG (aka I2C status)


static aoresult_t aoosp_con_seti2ccfg(aoosp_tele_t * tele, uint16_t addr, uint8_t flags, uint8_t speed ) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;
  if( flags & ~0x0F          ) return aoresult_osp_arg;
  if( speed & ~0x0F          ) return aoresult_osp_arg;
  if( speed==0               ) return aoresult_osp_arg;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x57; // SETI2CCFG
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = flags << 4 | speed;

  tele->data[4] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a SETI2CCFG telegram.
            Sets the I2C configuration/status of the addressed node.
    @param  addr
            The address to send the telegram to (unicast),
            (theoretically, use 0 for broadcast, or 3F0..3FE for group).
    @param  flags
            The 4 bit I2C configuration flags.
    @param  speed
            The 4 bit I2C bus speed (divisor).
    @return aoresult_ok if all ok, otherwise an error code.
    @note   The I2C configuration register also double as I2C status register.
            For example twelve bit addressing and speed are configuration settings,
            whereas interrupt, ack/nack, and busy are status flags.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_seti2ccfg(uint16_t addr, uint8_t flags, uint8_t speed ) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_seti2ccfg(&tele, addr, flags, speed);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("seti2ccfg(0x%03X,0x%02X,0x%02X)",addr,flags,speed);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 58 READOTP


static aoresult_t aoosp_con_readotp(aoosp_tele_t * tele, uint16_t addr, uint8_t otpaddr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;
  if( otpaddr > 0x1F         ) return aoresult_osp_arg;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x58; // READOTP
  if( respsize ) *respsize = 4+8; // otp row

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = otpaddr;

  tele->data[4] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readotp(aoosp_tele_t * tele, uint8_t * buf, int size) {
  // Set constants
  const uint8_t payloadsize = 8;
  // Check telegram consistency
  if( tele==0 || buf==0                        ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x58      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;
  if( size<1 || size>8                         ) return aoresult_osp_arg;

  // Get fields
  // OSP telegrams are big endian, C byte arrays are little endian, so reverse.
  for( int i=0; i<size; i++ ) buf[i] = tele->data[10-i];

  return aoresult_ok;
}


/*!
    @brief  Sends a READOTP telegram and receives its response.
            Asks the addressed node to respond with content bytes from OTP memory.
    @param  addr
            The address to send the telegram to (unicast).
    @param  otpaddr
            The address of the OTP memory.
    @param  buf
            Pointer to buffer to hold the retrieved bytes.
    @param  size
            The size of the buffer.
    @return aoresult_ok if all ok, otherwise an error code.
    @note   The telegram always retrieves 8 bytes from OTP; if size is smaller
            this function will copy fewer bytes from telegram to buf.
    @note   Addresses beyond the OTP size (beyond 0x1F for SAID) are
            read as 0x00 (so no wrap-around).
    @note   The read is not from the OTP, but from the OTP mirror in device RAM.
            The mirror is initialized with the OTP content on power on reset.
            However SETOTP writes to this RAM; then the mirror starts to differ from OTP.
    @note   The OTP access takes time, so wait 60us after sending this telegram.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_readotp(uint16_t addr, uint8_t otpaddr, uint8_t * buf, int size) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readotp(&tele,addr,otpaddr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readotp(&resp, buf, size);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readotp(0x%03X,0x%02X)",addr,otpaddr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" otp 0x%02X: %s\n", otpaddr, aoosp_prt_bytes(buf,size) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 59 SETOTP


// Notes on using SETOTP
// (1) SETOTP only works when the correct password is first sent using TESTPW.
// (2) SETOTP can only write blocks of 7 bytes, no more, no less.
// (3) SETOTP doesn't write to OTP, rather it writes to its shadow P2RAM.
// (4) P2RAM is initialized (copied) from OTP at startup.
// (5) P2RAM is non-persistent over power cycles.
// (6) P2RAM is persistent over a RESET telegram.
// (7) The I2C_EN (in OTP at 0D.0) is inspected by the SAID when sending it I2C telegrams.
// (8) The SPI_MODE (in OTP at 0D.3) is inspected by the SAID at startup, so P2RAM value is irrelevant.
// (9) At this moment I do not know how to flash P2RAM to OTP to make settings persistent.


static aoresult_t aoosp_con_setotp(aoosp_tele_t * tele, uint16_t addr, uint8_t otpaddr, uint8_t * buf, int size) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;
  if( otpaddr > 0x1F         ) return aoresult_osp_arg;
  if( size   != 7            ) return aoresult_osp_arg;

  // Set constants
  const uint8_t payloadsize = 8; // 1 for otp target address, 7 for data
  const uint8_t tid = 0x59; // SETOTP
  // if( respsize ) *respsize = 4+0; // nothing

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  // OSP telegrams are big endian, C byte arrays are little endian, so reverse
  for( int i=0; i<size; i++ ) tele->data[9-i] = buf[i];
  tele->data[10] = otpaddr;

  tele->data[11] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a SETOTP telegram.
            Writes bytes to the OTP memory of the addressed node.
    @param  addr
            The address to send the telegram to (unicast),
            (theoretically, use 0 for broadcast, or 3F0..3FE for group).
    @param  otpaddr
            The address of the OTP memory.
    @param  buf
            Pointer to buffer that holds the bytes to be written.
    @param  size
            The size of the buffer.
    @return aoresult_ok if all ok, otherwise an error code.
    @note   The telegram always writes 7 bytes from OTP, so
            it is advised to do read-modify-write on a 7 byte buffer.
    @note   Addresses beyond the OTP size (beyond 0x1F for SAID) are
            ignored for write (so no wrap-around).
    @note   The write is not to the OTP, but to the OTP mirror in device RAM.
            The mirror is initialized with the OTP content on power on reset.
            The mirror is not re-initialized by a RESET telegram.
    @note   SETOTP only works when the correct password is first sent using TESTPW.
            Without the TESPW set, the SETOTP does not update the OTP mirror.
            The TETSPW must be unset (eg set to 0) for normal operation,
            because in when the test password is set the node garbles
            forwarded telegrams.
    @note   The OTP access takes time, so wait 60us after sending this telegram.
    @note   See the high-level function aoosp_exec_setotp().
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_setotp(uint16_t addr, uint8_t otpaddr, uint8_t * buf, int size) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_setotp(&tele,addr,otpaddr,buf,size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("setotp(0x%03X,0x%02X,%s)",addr,otpaddr,aoosp_prt_bytes(buf,size) );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 5A READTESTDATA


// ==========================================================================
// Telegram 5B SETTESTDATA


static aoresult_t aoosp_con_settestdata(aoosp_tele_t * tele, uint16_t addr, uint16_t data ) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 2;
  const uint8_t tid = 0x5B; // SETTESTDATA
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = BITS_SLICE(data,8,16);
  tele->data[4] = BITS_SLICE(data,0,8);

  tele->data[5] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a SETTESTDATA telegram.
            Sets the test register of the addressed node.
    @param  addr
            The address to send the telegram to (unicast),
            (theoretically, use 0 for broadcast, or 3F0..3FE for group).
    @param  data
            The 16 bit test data.
    @return aoresult_ok if all ok, otherwise an error code.
    @note   This register is not for normal use;
            it contains the field to enter test mode, and to execute chip tests.
            Write only works when the correct password is first sent using TESTPW.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_settestdata(uint16_t addr, uint16_t data ) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_settestdata(&tele, addr, data);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("settestdata(0x%03X,0x%04X)",addr,data);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 5C READADCDAT


static aoresult_t aoosp_con_readadcdat(aoosp_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr) ) return aoresult_osp_addr;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x5C; // READADCDAT
  if( respsize ) *respsize = 4+2; // data

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_readadcdat(aoosp_tele_t * tele, uint16_t *adcdat ) {
  // Set constants
  const uint8_t payloadsize = 2;
  // Check telegram consistency
  if( tele==0 || adcdat==0                     ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x5C      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *adcdat= (tele->data[3] << 8) | (tele->data[4] << 0);

  return aoresult_ok;
}


/*!
    @brief  Sends a READADCDAT telegram and receives its response.
            Asks the addressed node to respond with its latest
            ADC measurement result.
    @param  addr
            The address to send the telegram to (unicast).
    @param  adcdat
            Output parameter returning the ADC measurement
            of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
            When returning aoresult_ok, the output parameter is set.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
    @note   The ADC must first be configured;
            see aoosp_send_setadc() for details.
    @note   To converting raw ADC to volt see `aoosp_prt_adc()`.

*/
aoresult_t aoosp_send_readadcdat(uint16_t addr, uint16_t *adcdat ) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_readadcdat(&tele,addr,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_readadcdat(&resp, adcdat);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("readadcdat(0x%03X)",addr);
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" adcdat=0x%04X=%dmV\n", *adcdat, aoosp_prt_adc(*adcdat) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================
// Telegram 5D TESTSCAN
// Telegram 5E -- no GETTESTPW


// ==========================================================================
// Telegram 5F SETTESTPW


static aoresult_t aoosp_con_settestpw(aoosp_tele_t * tele, uint16_t addr, uint64_t pw) {
  // Check input parameters
  if( tele==0                         ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr)          ) return aoresult_osp_addr;
  if( pw & ~0x0000FFFFFFFFFFFFULL     ) return aoresult_osp_arg;

#if AOOSP_LOG_ENABLED
  if( pw == AOOSP_SAID_TESTPW_UNKNOWN ) Serial.printf("WARNING: ask ams-OSRAM for TESTPW and see aoosp_said_testpw_get() for how to set it\n");
#endif

  // Set constants
  const uint8_t payloadsize = 6;
  const uint8_t tid = 0x5F; // SETTESTPW
  // if( respsize ) *respsize = 4+0; // nothing

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  memcpy( &(tele->data[3]), &pw, 6); // 3..8

  tele->data[9] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


/*!
    @brief  Sends a SETTESTPW telegram.
            Sets the password of the addressed node.
    @param  addr
            The address to send the telegram to (unicast),
            (theoretically, use 0 for broadcast, or 3F0..3FE for group).
    @param  pw
            The 48 bit password.
    @return aoresult_ok if all ok, otherwise an error code.
    @note   Ask ams-OSRAM for the password, see eg aoosp_said_testpw_get().
    @note   This register is not for normal use; it is needed to enter
            test mode (SETTESTDATA) for the manufacturer. The exception
            to the rule is that it is also needed to make SETOTP work.
    @note   The TETSPW must be unset (eg set to 0) for normal operation,
            because when the test password is set the node garbles
            forwarded telegrams.
    @note   Because of the impact on the node's behavior, it might be wise
            to use the variant with acknowledge (SETTESTPW_SR).
    @note   The term "test password" is a misnomer, with the correct password
            the host is authenticated, but not in test mode. The latter is
            a next step via SETTESTDATA.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_settestpw(uint16_t addr, uint64_t pw) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_settestpw(&tele,addr,pw);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_tx(tele.data,tele.size);
  if( spi_result!=aoresult_ok ) result= spi_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("settestpw(0x%03X,%s)",addr,aoosp_prt_bytes(&pw,6) );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
    Serial.printf("\n" );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}


// ==========================================================================

// Telegram 60 -- READSTAT with SR
// Telegram 61 -- no SETSTAT (with SR)
// Telegram 62 -- READTEMPSTAT with SR
// Telegram 63 -- no SETTEMPSTAT (with SR)
// Telegram 64 -- READCOMST with SR
// Telegram 65 -- no SETCOMST (with SR)
// Telegram 66 -- READLEDST  with SR
// Telegram 67 -- no SETLEDST (with SR)
// Telegram 68 -- READTEMP with SR
// Telegram 69 -- no SETTEMP (with SR)
// Telegram 6A -- no READOTTH with SR
// Telegram 6B SETOTTH_SR
// Telegram 6D SETSETUP_SR
// Telegram 6F SETPWM_SR
// Telegram 6F SETPWMCHN_SR
// Telegram 71 SETCURCHN_SR
// Telegram 73 SETTCOEFF_SR
// Telegram 75 SETADC_SR
// Telegram 77 SETI2CCFG_SR
// Telegram 79 SETOTP_SR


// ==========================================================================
// Telegram 7F SETTESTPW_SR


static aoresult_t aoosp_con_settestpw_sr(aoosp_tele_t * tele, uint16_t addr, uint64_t pw, uint8_t * respsize ) {
  // Check input parameters
  if( tele==0                         ) return aoresult_outargnull;
  if( !AOOSP_ADDR_ISOK(addr)          ) return aoresult_osp_addr;
  if( pw & ~0x0000FFFFFFFFFFFFULL     ) return aoresult_osp_arg;
#if AOOSP_LOG_ENABLED
  if( pw == AOOSP_SAID_TESTPW_UNKNOWN ) Serial.printf("WARNING: ask ams-OSRAM for TESTPW and see aoosp_said_testpw_get() for how to set it\n");
#endif

  // Set constants
  const uint8_t payloadsize = 6;
  const uint8_t tid = 0x7F; // SETTESTPW_SR
  if( respsize ) *respsize = 4+2; // temp,stat

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(SIZE2PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(SIZE2PSI(payloadsize),0,1)<<7 | tid;

  memcpy( &(tele->data[3]), &pw, 6); // 3..8

  tele->data[9] = aoosp_crc( tele->data , tele->size - 1 );

  return aoresult_ok;
}


static aoresult_t aoosp_des_settestpw_sr(aoosp_tele_t * tele, uint8_t * temp, uint8_t * stat) {
  // Set constants
  const uint8_t payloadsize = 2;
  // Check telegram consistency
  if( tele==0 || stat==0                       ) return aoresult_outargnull;
  if( tele->size!=4+payloadsize                ) return aoresult_osp_size;
  if( TELEPSI(tele)!=SIZE2PSI(payloadsize)     ) return aoresult_osp_psi;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return aoresult_osp_preamble;
  if( BITS_SLICE(tele->data[2],0,7)!=0x7F      ) return aoresult_osp_tid;
  if( aoosp_crc(tele->data,tele->size)!=0      ) return aoresult_osp_crc;

  // Get fields
  *temp = tele->data[3];
  *stat = tele->data[4];

  return aoresult_ok;
}


/*!
    @brief  Sends a SETTESTPW_SR telegram and receives a status response.
            Sets the password of the addressed node.
    @param  addr
            The address to send the telegram to (unicast),
            (theoretically, use 0 for broadcast, or 3F0..3FE for group).
    @param  pw
            The 48 bit password.
    @param  temp
            Output parameter returning the raw temperature of the addressed node.
    @param  stat
            Output parameter returning the (system) status of the addressed node.
    @return aoresult_ok if all ok, otherwise an error code.
    @note   Ask ams-OSRAM for the password, see eg aoosp_said_testpw_get().
    @note   This register is not for normal use; it is needed to enter
            test mode (SETTESTDATA) for the manufacturer. The exception
            to the rule is that it is also needed to make SETOTP work.
    @note   The TESTSPW must be unset (eg set to 0) for normal operation,
            because when the test password is set the node garbles
            forwarded telegrams.
    @note   The term "test password" is a misnomer, with the correct password
            the host is authenticated, but not in test mode. The latter is
            a next step via SETTESTDATA.
    @note   When logging enabled with aoosp_loglevel_set(), logs to Serial.
*/
aoresult_t aoosp_send_settestpw_sr(uint16_t addr, uint64_t pw, uint8_t * temp, uint8_t * stat) {
  // Telegram and result vars
  aoosp_tele_t tele;
  aoosp_tele_t resp;
  aoresult_t   result    = aoresult_ok;
  aoresult_t   con_result= aoresult_ok;
  aoresult_t   spi_result= aoresult_ok;
  aoresult_t   des_result= aoresult_ok;

  // Construct, send and optionally destruct
  if(     result==aoresult_ok ) con_result= aoosp_con_settestpw_sr(&tele,addr,pw,&resp.size);
  if( con_result!=aoresult_ok ) result=con_result;
  if(     result==aoresult_ok ) spi_result= aospi_txrx(tele.data,tele.size,resp.data,resp.size);
  if( spi_result!=aoresult_ok ) result= spi_result;
  if(     result==aoresult_ok ) des_result = aoosp_des_settestpw_sr(&resp, temp, stat);
  if( des_result!=aoresult_ok ) result=des_result;

  // Log
  #if AOOSP_LOG_ENABLED
  if( aoosp_loglevel >= aoosp_loglevel_args ) {
    Serial.printf("settestpw_sr(0x%03X,%s)",addr,aoosp_prt_bytes(&pw,6) );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [tele %s]",aoosp_prt_bytes(tele.data,tele.size));
    if( con_result!=aoresult_ok ) Serial.printf(" [constructor ERROR %s]", aoresult_to_str(con_result) );
      else if( spi_result!=aoresult_ok ) Serial.printf(" [SPI ERROR %s]", aoresult_to_str((aoresult_t)spi_result) );
      else if( des_result!=aoresult_ok ) Serial.printf(" [destructor ERROR %s]", aoresult_to_str(des_result) );
    Serial.printf(" ->" );
    if( aoosp_loglevel >= aoosp_loglevel_tele ) Serial.printf(" [resp %s]",aoosp_prt_bytes(resp.data,resp.size));
    Serial.printf(" temp=0x%02X=%d stat=0x%02X=%s", *temp, aoosp_prt_temp_said(*temp), *stat, aoosp_prt_stat_said(*stat) );
    Serial.printf(" (%d, %s)\n",  aoosp_prt_temp_rgbi(*temp), aoosp_prt_stat_rgbi(*stat) );
  }
  #endif // AOOSP_LOG_ENABLED

  return result;
}

