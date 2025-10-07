/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK4 Rev2 RAB5-OSIRE E3731i
*              temperature compensation example for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2025-07-21
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "osp/aospi/aospi.h"
#include "osp/aoosp/aoosp_send.h"
#include "osp/aomw/aomw_color.h"
#include "hal/osp_spi.h"
#include "osp/aomw/aomw_otp.h"
#include "osp/aoosp/aoosp_prt.h"

/**
 * @def OSP_NODE_COUNT
 * @brief For this demo, we use the RAB5-OSIRE board with 5 nodes
 * 4 are E3731i LEDs and 1 (address = 3) is an AS1163 (SAID driver)
 */
#define OSP_NODE_COUNT 5

int main(void)
{
	cy_rslt_t result;

	// Initialize the device and board peripherals
	result = cybsp_init();
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	// Enable debug output via KitProg UART
	result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, 115200);
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	__enable_irq();

	printf("RDK4 - RAB5-OSIRE E3731i temperature compensation example \r\n");

	// Initialize AOSPI layer
	// MCU-A -> we are using only SPI_MOSI (no CLK) therefore the communication is Manchester encoded
	aoresult_t osp_result = aospi_init(aospi_phy_mcua, osp_spi_init, osp_spi_tx, osp_spi_txrx);
	printf("aospi_init -> %s\r\n", aoresult_to_str(osp_result, 1));
	if (osp_result != aoresult_ok)
	{
		for(;;){}
	}

	// Send reset to all OSP nodes (0 for broadcast)
	osp_result = aoosp_send_reset(0);
	printf("aoosp_send_reset -> %s\r\n", aoresult_to_str(osp_result, 1));
	if (osp_result != aoresult_ok) for(;;){}

	// Delay time after reset
	Cy_SysLib_Delay(1);

	// Assumption: BIDIR mode
	uint16_t last_addr = 0;
	uint8_t node_temperature = 0;
	uint8_t node_status = 0;
	osp_result = aoosp_send_initbidir(0x1, &last_addr, &node_temperature, &node_status);
	printf("aoosp_send_initbidir -> %s\r\n", aoresult_to_str(osp_result, 1));
	if (osp_result != aoresult_ok) for(;;){}

	printf("Last node temperature = %d \t and status = %d \r\n", node_temperature, node_status);
	printf("Temperature = %d\r\n", aoosp_prt_temp_rgbi(node_temperature));
	printf("Status = %s\r\n", aoosp_prt_stat_state(node_status));

	// Clear error (broadcast)
	osp_result = aoosp_send_clrerror(0x0);
	printf("aoosp_send_clrerror -> %s\r\n", aoresult_to_str(osp_result, 1));

	// Go active (at start, the OSP devices are in sleep mode)
	osp_result = aoosp_send_goactive(0);
	printf("aoosp_send_goactive -> %s\r\n", aoresult_to_str(osp_result, 1));

	// Check the status of each node (should be active now)
	for(uint16_t i = 0; i < last_addr; ++i)
	{
		uint16_t node_addr = (i+1);
		uint8_t node_status = 0;
		osp_result = aoosp_send_readstat(node_addr, &node_status);

		printf("aoosp_send_readstat [%d] -> %s\r\n", node_addr, aoresult_to_str(osp_result, 1));
		printf("Status = %s\r\n", aoosp_prt_stat_state(node_status));
	}

	// For this example, expect OSP_NODE_COUNT
	if (last_addr != OSP_NODE_COUNT)
	{
		printf("We expect %d nodes for this example, but we discovered %d nodes...\r\n", OSP_NODE_COUNT, last_addr);
		for(;;){}
	}

	// We assume that we have following architecture:
	// Index	| OSP node type
	// 1		| E3731i
	// 2		| E3731i
	// 3		| AS1163 SAID driver
	// 4		| E3731i
	// 5 		| E3731i
	// We will read the content for 1, 2, 4 and 5
	aomw_otp_rgbi_t nodes_memory_content [OSP_NODE_COUNT] = {0};
	uint8_t nodes_type[OSP_NODE_COUNT] = {0};
	uint8_t nodes_setup_flags[OSP_NODE_COUNT] = {0};

	for(uint16_t i = 0; i < last_addr; ++i)
	{
		uint16_t node_addr = (i+1);
		uint32_t node_id = 0;
		osp_result = aoosp_send_identify(node_addr, &node_id);
		printf("aoosp_send_identify [%d] -> %s\r\n", node_addr, aoresult_to_str(osp_result, 1));
		if (osp_result != aoresult_ok) for(;;){}

		uint16_t part_id = (uint16_t) AOOSP_IDENTIFY_ID2PART(node_id);

		// Store the type (E3731i or SAID)
		nodes_type[i] = part_id;

		if (part_id == AOOSP_IDENTIFY_MANUPART_RGBI)
		{
			// E3731i -> read OTP content
			osp_result = aomw_read_otp_rgbi(node_addr, &nodes_memory_content[i]);
			printf("aomw_read_otp_rgbi [%d] -> %s\r\n", node_addr, aoresult_to_str(osp_result, 1));
			if (osp_result != aoresult_ok) for(;;){}

			// Read setup flags -> needed to know the PWM resolution
			osp_result = aoosp_send_readsetup(node_addr, &nodes_setup_flags[i]);
			printf("aoosp_send_readsetup [%d] -> %s\r\n", node_addr, aoresult_to_str(osp_result, 1));
			if (osp_result != aoresult_ok) for(;;){}
		}
		else if (part_id == AOOSP_IDENTIFY_MANUPART_SAID)
		{
			printf("SAID AS1163 driver - do not read memory. \r\n");
			nodes_memory_content[i].wafer_number = 0;
			nodes_memory_content[i].chip_id = 0;
		}
	}

	// LSBu, LSBv and LSBIv are defined inside OSIRE-E3731i-Startup-Guide.pdf
	static const float lsbiv_mcd_nighttime = 0.44f; // mcd
	static const float lsbu_nighttime = 0.0025f;
	static const float lsbv_nighttime = 0.0025f;

	// Convert (only for Night time)
	aomw_color_uprime_vprime_iv3_t nodes_calibration[OSP_NODE_COUNT] = {0};
	for(uint16_t i = 0; i < last_addr; ++i)
	{
		nodes_calibration[i].r.u_prime	= nodes_memory_content[i].red_u_prime_night * lsbu_nighttime;
		nodes_calibration[i].r.v_prime	= nodes_memory_content[i].red_v_prime_night * lsbv_nighttime;
		nodes_calibration[i].r.iv		= nodes_memory_content[i].red_iv_night * lsbiv_mcd_nighttime / 1000.f;

		nodes_calibration[i].g.u_prime	= nodes_memory_content[i].green_u_prime_night * lsbu_nighttime;
		nodes_calibration[i].g.v_prime	= nodes_memory_content[i].green_v_prime_night * lsbv_nighttime;
		nodes_calibration[i].g.iv		= nodes_memory_content[i].green_iv_night * lsbiv_mcd_nighttime / 1000.f;

		nodes_calibration[i].b.u_prime	= nodes_memory_content[i].blue_u_prime_night * lsbu_nighttime;
		nodes_calibration[i].b.v_prime	= nodes_memory_content[i].blue_v_prime_night * lsbv_nighttime;
		nodes_calibration[i].b.iv		= nodes_memory_content[i].blue_iv_night * lsbiv_mcd_nighttime / 1000.f;
	}

	// Select target in the CIE 1931 space (X, Y, IV)
	aomw_color_cxcyiv1_t user_target_cxcyiv1= {0.40, 0.20, 0.1}; // chosen target [**] color: purple

	// Convert it into tristimulus
	aomw_color_xyz1_t user_target;
	aomw_color_cxcyiv1_to_xyz1( &user_target_cxcyiv1, &user_target );

	// Temperature compensation constants
	// see https://look.ams-osram.com/m/3ba9f6b3c2dd7647/original/OSIRE-E3731i-Startup-Guide.pdf page 20/51
	static aomw_color_iv_uprime_vprime_poly3_t nightime_e3731i_poly = {
	  .r={.Iv={.a=  1.030E-05,.b=-7.397E-03},.Uprime={.a=-1.653E-06,.b= 4.029E-04},.Vprime={.a= 2.567E-07,.b=-6.984E-05}},
	  .g={.Iv={.a= -5.864E-07,.b=-2.306E-03},.Uprime={.a= 4.555E-06,.b= 2.081E-03},.Vprime={.a=-8.166E-08,.b=-1.921E-05}},
	  .b={.Iv={.a= -2.679E-06,.b= 7.200E-04},.Uprime={.a=-1.060E-06,.b=-6.739E-04},.Vprime={.a= 5.774E-06,.b= 2.434E-03}}
	};

	// Constants for day time
	// We do not use it in this example, therefore commented out
//	static aomw_color_iv_uprime_vprime_poly3_t daytime_e3731i_poly = {
//	  .r={.Iv={.a= 1.122E-05,.b=-7.808E-03},.Uprime={.a=-1.466E-06,.b= 3.727E-04},.Vprime={.a= 2.475E-07,.b=-6.384E-05}},
//	  .g={.Iv={.a=-5.946E-06,.b=-1.813E-03},.Uprime={.a= 5.878E-06,.b= 2.241E-03},.Vprime={.a=-1.897E-07,.b= 1.981E-05}},
//	  .b={.Iv={.a=-2.226E-06,.b= 7.734E-04},.Uprime={.a=-1.036E-06,.b=-6.427E-04},.Vprime={.a= 6.519E-06,.b= 2.423E-03}}
//	};

	// Endless loop
	// For each LED:
	// Read temperature, compute new PWM, apply new PWM values
	for(;;)
	{
		for(uint16_t i = 0; i < last_addr; ++i)
		{
			// Only for E3731i types
			if (nodes_type[i] == AOOSP_IDENTIFY_MANUPART_RGBI)
			{
				uint16_t node_addr = (i+1);
				uint8_t temperature_raw = 0;
				uint16_t error_counter = 0;

				// Remark: on RDK4, because switching from SPI master to SPI slave takes sometime to much time
				// reading the temperature of the first node (because very fast response time) sometimes fails
				// Therefore, read temperature in a loop
				for(;;)
				{
					osp_result = aoosp_send_readtemp(node_addr, &temperature_raw);
					if (osp_result != aoresult_ok) error_counter++;
					else break;
					if (error_counter > 100)
					{
						printf("aoosp_send_readtemp [%d] -> %s\r\n", node_addr, aoresult_to_str(osp_result, 1));
						for(;;){}
					}
				}

				printf("Temperature %d -> %d \r\n", node_addr, aoosp_prt_temp_rgbi(temperature_raw));

				aomw_color_uprime_vprime_iv3_t node_calib = nodes_calibration[i];

				// Apply temperature polynom to it
				aomw_color_poly_ivuprimevprime_apply3(&node_calib, &nightime_e3731i_poly, ((float)temperature_raw - 140.f));

				// Convert them to tristimulus matrix
				aomw_color_xyz3_t tristimulus_matrix;
				aomw_color_uprime_vprime_iv3_to_xyz3(&node_calib, &tristimulus_matrix);

				// Compute color mix
				aomw_color_mix_t color_mix;
				aomw_color_computemix(&tristimulus_matrix, &user_target, &color_mix);

				// Compute PWM values
				int max_pwm = 0;
				// OSIRE-E3731i-Startup-Guide.pdf page 33/51
				if (nodes_setup_flags[i] & AOOSP_SETUP_FLAGS_PWMF)
				{
					// 0b1 -> 1172 Hz pwm frequency, 14 bit
					max_pwm = 0x3FFF; // 2^14-1
				}
				else
				{
					// 0b0 -> 586 Hz pwm frequency, 15 bit
					max_pwm = 0x7FFF; // 2^15-1
				}

				aomw_color_pwm_t pwm_output;
				aomw_color_mix_to_pwm(&color_mix, max_pwm, &pwm_output);

				printf("pwm_output: Addr = %d: R = %d, G = %d, B = %d\r\n", node_addr, pwm_output.r, pwm_output.g, pwm_output.b);

				// Apply
				osp_result = aoosp_send_setpwm(node_addr, pwm_output.r, pwm_output.g, pwm_output.b, 0);
				if (osp_result != aoresult_ok)
				{
					printf("aoosp_send_setpwm [%d] -> %s\r\n", node_addr, aoresult_to_str(osp_result, 1));
					for(;;){}
				}
			}
		}

		// Toggle Blue LED
		Cy_GPIO_Clr(USER_LED_BLUE_PORT, USER_LED_BLUE_PIN);
		Cy_SysLib_Delay(5);
		Cy_GPIO_Set(USER_LED_BLUE_PORT, USER_LED_BLUE_PIN);

		// Delay
		Cy_SysLib_Delay(1000);
	}
}

/* [] END OF FILE */
