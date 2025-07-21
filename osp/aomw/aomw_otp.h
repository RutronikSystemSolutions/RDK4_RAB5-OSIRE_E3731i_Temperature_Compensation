/*
 * aomw_otp.h
 *
 *  Created on: Jul 11, 2025
 *      Author: ROJ030
 */

#ifndef _AOMW_AOMW_OTP_H_
#define _AOMW_AOMW_OTP_H_

#include <stdint.h>

#include "../aoresult/aoresult.h"

typedef struct aomw_otp_rgbi_s
{
	uint16_t chip_id;
	uint8_t wafer_number;

	uint8_t red_u_prime_night;
	uint8_t red_v_prime_night;
	uint16_t red_iv_night;

	uint8_t red_u_prime_day;
	uint8_t red_v_prime_day;
	uint16_t red_iv_day;

	uint8_t blue_u_prime_night;
	uint8_t blue_v_prime_night;
	uint16_t blue_iv_night;

	uint8_t blue_u_prime_day;
	uint8_t blue_v_prime_day;
	uint16_t blue_iv_day;

	uint8_t green_u_prime_night;
	uint8_t green_v_prime_night;
	uint16_t green_iv_night;

	uint8_t green_u_prime_day;
	uint8_t green_v_prime_day;
	uint16_t green_iv_day;
} aomw_otp_rgbi_t;

// Read and convert the content of the OTP memory of RGBi LED
aoresult_t aomw_read_otp_rgbi(uint16_t addr, aomw_otp_rgbi_t* otp_content);

#endif /* _AOMW_AOMW_OTP_H_ */
