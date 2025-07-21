// aospi.h - 2-wire SPI (and 1-wire Manchester) towards and from OSP nodes
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
#ifndef _AOSPI_H_
#define _AOSPI_H_

#include <stdint.h>   // uint8_t etc
#include "../aoresult/aoresult.h"

// Identifies lib version
#define AOSPI_VERSION "1.0.0"


// OSP uses telegrams of max 12 bytes
#define AOSPI_TELE_MAXSIZE 12

// This library supports multiple physical layers
typedef enum aospi_phy_e {
  aospi_phy_undef, // None selected yet
  aospi_phy_mcua,  // MCU mode Type A -> only the SIO1_P input is used. Manchester encoded signal
  aospi_phy_mcub,  // MCU mode Type B -> only possible is first node is a AS1163
} aospi_phy_t;

// Callback function called to init spi communication
typedef int (*osp_spi_init_func_t)(void);

// Callback function called to send something over SPI
typedef int (*osp_spi_tx_func_t)(uint8_t *tx, int size);

// Callback function called to send and receive something over SPI
typedef int (*osp_spi_txrx_func_t)(uint8_t *tx, int tx_size, uint8_t *rx, int rx_size);

// Sends the `txsize` bytes in buffer `tx` to the first OSP node.
aoresult_t aospi_tx(const uint8_t * tx, int txsize);

// Sends the `txsize` bytes in buffer `tx` to the first OSP node. Waits for a response telegram and stores those bytes in buffer `rx` with size `rxsize`.
aoresult_t aospi_txrx(const uint8_t *tx, int txsize, uint8_t *rx, int rxsize);

aoresult_t aospi_init(aospi_phy_t phy, osp_spi_init_func_t init_func, osp_spi_tx_func_t tx_func, osp_spi_txrx_func_t txrx_func);

#endif
