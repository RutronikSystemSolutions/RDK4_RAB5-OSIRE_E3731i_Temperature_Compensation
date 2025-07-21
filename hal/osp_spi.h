/*
 * osp_spi.h
 *
 *  Created on: Jul 9, 2025
 *      Author: ROJ030
 */

#ifndef HAL_OSP_SPI_H_
#define HAL_OSP_SPI_H_

#include <stdint.h>

int osp_spi_init();

int osp_spi_tx(uint8_t *tx, int size);

int osp_spi_txrx(uint8_t *tx, int tx_size, uint8_t *rx, int rx_size);

#endif /* HAL_OSP_SPI_H_ */
