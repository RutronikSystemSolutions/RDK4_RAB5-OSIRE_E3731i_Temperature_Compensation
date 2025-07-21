/*
 * aospi.c
 *
 *  Created on: Jul 9, 2025
 *      Author: ROJ030
 */

#include "aospi.h" // own API

#include <stddef.h> // NULL

// The lookup table, mapping 8 payload bits to 16 Manchester'ed bits
static const uint16_t aospi_manchester[] = {
  /*00*/ 0xAAAA, 0xAAA9, 0xAAA6, 0xAAA5, 0xAA9A, 0xAA99, 0xAA96, 0xAA95,
  /*08*/ 0xAA6A, 0xAA69, 0xAA66, 0xAA65, 0xAA5A, 0xAA59, 0xAA56, 0xAA55,
  /*10*/ 0xA9AA, 0xA9A9, 0xA9A6, 0xA9A5, 0xA99A, 0xA999, 0xA996, 0xA995,
  /*18*/ 0xA96A, 0xA969, 0xA966, 0xA965, 0xA95A, 0xA959, 0xA956, 0xA955,
  /*20*/ 0xA6AA, 0xA6A9, 0xA6A6, 0xA6A5, 0xA69A, 0xA699, 0xA696, 0xA695,
  /*28*/ 0xA66A, 0xA669, 0xA666, 0xA665, 0xA65A, 0xA659, 0xA656, 0xA655,
  /*30*/ 0xA5AA, 0xA5A9, 0xA5A6, 0xA5A5, 0xA59A, 0xA599, 0xA596, 0xA595,
  /*38*/ 0xA56A, 0xA569, 0xA566, 0xA565, 0xA55A, 0xA559, 0xA556, 0xA555,
  /*40*/ 0x9AAA, 0x9AA9, 0x9AA6, 0x9AA5, 0x9A9A, 0x9A99, 0x9A96, 0x9A95,
  /*48*/ 0x9A6A, 0x9A69, 0x9A66, 0x9A65, 0x9A5A, 0x9A59, 0x9A56, 0x9A55,
  /*50*/ 0x99AA, 0x99A9, 0x99A6, 0x99A5, 0x999A, 0x9999, 0x9996, 0x9995,
  /*58*/ 0x996A, 0x9969, 0x9966, 0x9965, 0x995A, 0x9959, 0x9956, 0x9955,
  /*60*/ 0x96AA, 0x96A9, 0x96A6, 0x96A5, 0x969A, 0x9699, 0x9696, 0x9695,
  /*68*/ 0x966A, 0x9669, 0x9666, 0x9665, 0x965A, 0x9659, 0x9656, 0x9655,
  /*70*/ 0x95AA, 0x95A9, 0x95A6, 0x95A5, 0x959A, 0x9599, 0x9596, 0x9595,
  /*78*/ 0x956A, 0x9569, 0x9566, 0x9565, 0x955A, 0x9559, 0x9556, 0x9555,
  /*80*/ 0x6AAA, 0x6AA9, 0x6AA6, 0x6AA5, 0x6A9A, 0x6A99, 0x6A96, 0x6A95,
  /*88*/ 0x6A6A, 0x6A69, 0x6A66, 0x6A65, 0x6A5A, 0x6A59, 0x6A56, 0x6A55,
  /*90*/ 0x69AA, 0x69A9, 0x69A6, 0x69A5, 0x699A, 0x6999, 0x6996, 0x6995,
  /*98*/ 0x696A, 0x6969, 0x6966, 0x6965, 0x695A, 0x6959, 0x6956, 0x6955,
  /*A0*/ 0x66AA, 0x66A9, 0x66A6, 0x66A5, 0x669A, 0x6699, 0x6696, 0x6695,
  /*A8*/ 0x666A, 0x6669, 0x6666, 0x6665, 0x665A, 0x6659, 0x6656, 0x6655,
  /*B0*/ 0x65AA, 0x65A9, 0x65A6, 0x65A5, 0x659A, 0x6599, 0x6596, 0x6595,
  /*B8*/ 0x656A, 0x6569, 0x6566, 0x6565, 0x655A, 0x6559, 0x6556, 0x6555,
  /*C0*/ 0x5AAA, 0x5AA9, 0x5AA6, 0x5AA5, 0x5A9A, 0x5A99, 0x5A96, 0x5A95,
  /*C8*/ 0x5A6A, 0x5A69, 0x5A66, 0x5A65, 0x5A5A, 0x5A59, 0x5A56, 0x5A55,
  /*D0*/ 0x59AA, 0x59A9, 0x59A6, 0x59A5, 0x599A, 0x5999, 0x5996, 0x5995,
  /*D8*/ 0x596A, 0x5969, 0x5966, 0x5965, 0x595A, 0x5959, 0x5956, 0x5955,
  /*E0*/ 0x56AA, 0x56A9, 0x56A6, 0x56A5, 0x569A, 0x5699, 0x5696, 0x5695,
  /*E8*/ 0x566A, 0x5669, 0x5666, 0x5665, 0x565A, 0x5659, 0x5656, 0x5655,
  /*F0*/ 0x55AA, 0x55A9, 0x55A6, 0x55A5, 0x559A, 0x5599, 0x5596, 0x5595,
  /*F8*/ 0x556A, 0x5569, 0x5566, 0x5565, 0x555A, 0x5559, 0x5556, 0x5555,
};

static osp_spi_init_func_t internal_init = NULL;
static osp_spi_tx_func_t internal_tx = NULL;
static osp_spi_txrx_func_t internal_txrx = NULL;

static aospi_phy_t aospi_phy = aospi_phy_undef;

// Encodes the count bytes in incoming buffer bufi using the IEEE 802.4
// Manchester standard, and writes the encoded stream to bufo.
// Note bufo needs to be twice as big as bufi (not checked).
static void aospi_manchester_encode(const uint8_t *bufi, int count, uint8_t *bufo )
{
	for (uint8_t i = 0; i < count; i++)
	{
		uint16_t w = aospi_manchester[bufi[i]];
		*bufo++ = w >> 8; // transmission is big endian
		*bufo++ = w & 0xFF;
	}
}

/*!
    @brief  Sends the `txsize` bytes in buffer `tx` to the first OSP node,
	        using the selected physical layer.
    @param  tx
            A pointer to a buffer of bytes to be sent.
    @param  txsize
            The number of bytes (of buffer tx) to be sent.
    @return aoresult_spi_buf if tx is NULL
            aoresult_spi_buf if txsize is out of bounds (0..12)
	          aoresult_ok (no error checking on send possible)
    @note   With `aospi_init()` the physical layer is selected.
	          This function just dispatches to the actual implementation.
*/
aoresult_t aospi_tx(const uint8_t * tx, int txsize)
{
	// Parameter checks
	if( txsize<0 || txsize>AOSPI_TELE_MAXSIZE ) return aoresult_spi_buf;
	if( tx==0 )  return aoresult_spi_buf;

	switch(aospi_phy)
	{
		case aospi_phy_mcua:
			uint8_t tx_man[AOSPI_TELE_MAXSIZE*2];
			aospi_manchester_encode(tx,txsize,tx_man);
			if (internal_tx((uint8_t*)tx_man, txsize*2) != 0)
			{
				return aoresult_other;
			}
			break;
		case aospi_phy_mcub:
			if (internal_tx((uint8_t*)tx, txsize) != 0)
			{
				return aoresult_other;
			}
			break;
		default :
		      return aoresult_assert;
	}

	return aoresult_ok;
}

/*!
    @brief  Sends the `txsize` bytes in buffer `tx` to the first OSP node,
	          using the selected physical layer. Waits for a response telegram
            and stores those bytes in buffer `rx` with size `rxsize`.
    @param  tx
            A pointer to a buffer of bytes to be sent.
    @param  txsize
            The number of bytes (of buffer `tx`) to be sent.
    @param  rx
            A pointer to a caller allocated buffer of bytes to be received.
    @param  rxsize
            The size of the `rx` buffer.
    @return aoresult_spi_buf     if tx or rx is NULL
            aoresult_spi_buf     if txsize or rxsize is out of bounds (0..12)
            aoresult_assert      if the underlying driver behaves unexpectedly
            aoresult_spi_noclock if no response (clock) is received
            aoresult_spi_length  if the response had wrong number of bytes
            aoresult_ok          otherwise
    @note   Before sending, configure whether reception is from first (BiDir)
            or last (Loop) node in the OSP chain using aospi_dirmux_set_xxx()
    @note   Output parameter `actsize` might be set to NULL. In this case the
            function will not perform a size test, will thus not return
            aoresult_spi_length. Instead the caller can inspect `*actsize`.
    @note   If caller knows how many bytes will be received, set `rxsize` to
            that amount and set `actsize` pointer to NULL.
    @note   If caller does not knows how many bytes will be received, set
            `rxsize` to largest possible telegram (ie AOSPI_TELE_MAXSIZE)
            and pass an `actsize`.
    @note   With `aospi_init()` the physical layer is selected.
	          This function just dispatches to the actual implementation.
            Recall that the physical layer is only different fro the transmit
            part, reception is always 2-wire SPI.
*/
aoresult_t aospi_txrx(const uint8_t *tx, int txsize, uint8_t *rx, int rxsize)
{
	// Parameter checks
	if (txsize < 0 || txsize > AOSPI_TELE_MAXSIZE)
		return aoresult_spi_buf;
	if (tx == 0)
		return aoresult_spi_buf;

	switch(aospi_phy)
	{
		case aospi_phy_mcua:
			uint8_t tx_man[AOSPI_TELE_MAXSIZE * 2];
			aospi_manchester_encode(tx, txsize, tx_man);
			if (internal_txrx((uint8_t*)tx_man, txsize*2, rx, rxsize) != 0)
			{
				return aoresult_other;
			}
			break;

		case aospi_phy_mcub:
			if (internal_txrx((uint8_t*)tx, txsize, rx, rxsize) != 0)
			{
				return aoresult_other;
			}
			break;

		default :
			return aoresult_assert;
	}

	return aoresult_ok;
}

aoresult_t aospi_init(aospi_phy_t phy, osp_spi_init_func_t init_func, osp_spi_tx_func_t tx_func, osp_spi_txrx_func_t txrx_func)
{
	if ((phy != aospi_phy_mcua) && (phy != aospi_phy_mcub))
		return aoresult_other;

	// Store
	internal_init = init_func;
	internal_tx = tx_func;
	internal_txrx = txrx_func;
	aospi_phy = phy;

	if ((internal_init == NULL)
			|| (internal_tx == NULL)
			|| (internal_txrx == NULL))
		return aoresult_other;

	// Call to init
	if (internal_init() != 0)
		return aoresult_other;

	return aoresult_ok;
}
