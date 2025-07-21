/*
 * osp_spi.c
 *
 *  Created on: Jul 9, 2025
 *      Author: ROJ030
 */

#include "osp_spi.h"

#include "cy_pdl.h"
#include "cycfg.h"

#include "hal/hal_timer.h"

/**
 * @brief Store the SPI contexts
 */
static cy_stc_scb_spi_context_t master_spi_context;
static cy_stc_scb_spi_context_t slave_spi_context;

/**
 * @brief Initialize the SPI master used to control the LEDs
 * Configuration used: design.modus file
 *
 * @retval 0 Success / != 0 Error
 */
static int init_spi_master(void)
{
    cy_en_scb_spi_status_t result;

    // Configure the SPI block
    result = Cy_SCB_SPI_Init(mSPI_HW, &mSPI_config, &master_spi_context);
    if( result != CY_SCB_SPI_SUCCESS)
    {
        return -1;
    }

    // Set active slave select to line 0
    Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, CY_SCB_SPI_SLAVE_SELECT0);

    // Enable the SPI Master block
    Cy_SCB_SPI_Enable(mSPI_HW);

    return 0;
}

static void slave_spi_interrupt(void)
{
    Cy_SCB_SPI_Interrupt(sSPI_HW, &slave_spi_context);
}

static int init_spi_slave(void)
{
    cy_en_scb_spi_status_t spi_status;
    cy_en_sysint_status_t intr_status;
    const uint32_t sSPI_INTR_PRIORITY = 3;

    // Configure the SPI block
    spi_status = Cy_SCB_SPI_Init(sSPI_HW, &sSPI_config, &slave_spi_context);
    if(spi_status != CY_SCB_SPI_SUCCESS)
    {
        return -1;
    }

    // Populate configuration structure
    const cy_stc_sysint_t spi_intr_config =
    {
        .intrSrc      = sSPI_IRQ,
        .intrPriority = sSPI_INTR_PRIORITY,
    };

    // Hook interrupt service routine and enable interrupt
    intr_status = Cy_SysInt_Init(&spi_intr_config, &slave_spi_interrupt);

    if(intr_status != CY_SYSINT_SUCCESS)
    {
        return -2;
    }

    NVIC_EnableIRQ(sSPI_IRQ);

    // Enable the SPI Slave block
    Cy_SCB_SPI_Enable(sSPI_HW);

    return 0;
}

int osp_spi_init()
{
	if (init_spi_master() != 0)
	{
		return -1;
	}

	if (init_spi_slave() != 0)
	{
		return -2;
	}

	if (hal_timer_init() != 0)
	{
		return -3;
	}

	return 0;
}

int osp_spi_tx(uint8_t *tx, int size)
{
    Cy_SCB_SPI_Enable(mSPI_HW);
    // Initiate SPI Master write transaction
    Cy_SCB_SPI_WriteArrayBlocking(mSPI_HW, tx, size);
    // Blocking wait for transfer completion
    while (!Cy_SCB_SPI_IsTxComplete(mSPI_HW))
    {
    }
    Cy_SCB_SPI_Disable(mSPI_HW,&master_spi_context);
    return 0;
}

int osp_spi_txrx(uint8_t *tx, int tx_size, uint8_t *rx, int rx_size)
{
	static const uint32_t TIMEOUT_MS = 100;
	static const uint32_t TIMEOUT_NO_NEW_MS = 1;

	// First send
	if (osp_spi_tx(tx, tx_size) != 0)
	{
		return -1;
	}

	// Receive
	// CS for slave
	Cy_GPIO_Write(CS_SW_OUTPUT_PORT, CS_SW_OUTPUT_PIN, 0);

	// Start timer (timeout detection)
	hal_timer_start();

	uint32_t last_bytes_in_rx = 0;
	for(;;)
	{
		uint32_t bytes_in_rx = Cy_SCB_SPI_GetNumInRxFifo(sSPI_HW);

		if (bytes_in_rx >= rx_size)
			break;

		if ((bytes_in_rx != last_bytes_in_rx))
			hal_timer_restart();

		if ((bytes_in_rx == 0) && (hal_timer_get_ms() >= TIMEOUT_MS))
			break;

		if ((last_bytes_in_rx != 0)
				&& (bytes_in_rx == last_bytes_in_rx)
				&& (hal_timer_get_ms() >= TIMEOUT_NO_NEW_MS))
			break;

		last_bytes_in_rx = bytes_in_rx;
	}

	hal_timer_stop();

	// Enough bytes in the RX FIFO, read
	if (Cy_SCB_SPI_ReadArray(sSPI_HW, rx, rx_size) != rx_size)
	{
		// release CS for slave
		Cy_GPIO_Write(CS_SW_OUTPUT_PORT, CS_SW_OUTPUT_PIN, 1);
		return -2;
	}

	// release CS for slave
	Cy_GPIO_Write(CS_SW_OUTPUT_PORT, CS_SW_OUTPUT_PIN, 1);

	return 0;
}


