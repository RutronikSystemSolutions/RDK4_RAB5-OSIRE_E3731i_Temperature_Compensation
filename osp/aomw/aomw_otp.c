/*
 * aomw_otp.c
 *
 *  Created on: Jul 11, 2025
 *      Author: ROJ030
 */

#include "aomw_otp.h" // own API

#include "../aoosp/aoosp_send.h" // own API

/**
 * @def AOMW_OTPSIZE
 * @brief Size of the OTP memory
 */
#define AOMW_OTPSIZE 0x20

// Generic telegram field access macros
#define BITS_MASK(n)                  ((1<<(n))-1)                           // series of n bits: BITS_MASK(3)=0b111 (max n=31)
#define BITS_SLICE(v,lo,hi)           ( ((v)>>(lo)) & BITS_MASK((hi)-(lo)) ) // takes bits [lo..hi) from v: BITS_SLICE(0b11101011,2,6)=0b1010

aoresult_t aomw_read_otp_rgbi(uint16_t addr, aomw_otp_rgbi_t* otp_content)
{
	// See file https://look.ams-osram.com/m/3ba9f6b3c2dd7647/original/OSIRE-E3731i-Startup-Guide.pdf
	// to see the OTP map

	uint8_t otp_memory[AOMW_OTPSIZE] = {0};
	static const uint8_t OTPSTEP = 8;

	// Read OTP memory (by chunk of OTPSTEP = 8 bytes)
	for( uint8_t otpaddr = 0; otpaddr < AOMW_OTPSIZE; otpaddr += OTPSTEP )
	{
		aoresult_t osp_result = aoosp_send_readotp(addr, otpaddr, otp_memory + otpaddr, OTPSTEP);
		if (osp_result != aoresult_ok) return osp_result;
	}

	// Convert to structure content
	otp_content->chip_id = ((uint16_t)((otp_memory[0x00]) & 0xFF)) | ((((uint16_t)(otp_memory[0x01]) << 8) & 0xFF00));
	otp_content->wafer_number = otp_memory[0x02] & 0x1F;

	otp_content->red_u_prime_night = otp_memory[0x0A];
	otp_content->red_v_prime_night = otp_memory[0x0B];
	otp_content->red_iv_night = ((uint16_t)((otp_memory[0x0C]) & 0xFF)) | ((uint16_t)(otp_memory[0x0D] & 0x0F)) << 8;

	otp_content->red_u_prime_day = otp_memory[0x0E];
	otp_content->red_v_prime_day = otp_memory[0x0F];
	otp_content->red_iv_day = ((uint16_t)((otp_memory[0x10]) & 0xFF)) | ((uint16_t)(otp_memory[0x0D] & 0xF0)) << 4;

	otp_content->blue_u_prime_night = otp_memory[0x11];
	otp_content->blue_v_prime_night = otp_memory[0x12];
	otp_content->blue_iv_night = ((uint16_t)((otp_memory[0x13]) & 0xFF)) | ((uint16_t)(otp_memory[0x14] & 0x0F)) << 8;

	otp_content->blue_u_prime_day = otp_memory[0x15];
	otp_content->blue_v_prime_day = otp_memory[0x16];
	otp_content->blue_iv_day = ((uint16_t)((otp_memory[0x17]) & 0xFF)) | ((uint16_t)(otp_memory[0x14] & 0xF0)) << 4;

	otp_content->green_u_prime_night = otp_memory[0x18];
	otp_content->green_v_prime_night = otp_memory[0x19];
	otp_content->green_iv_night = ((uint16_t)((otp_memory[0x1A]) & 0xFF)) | ((uint16_t)(otp_memory[0x1B] & 0x0F)) << 8;

	otp_content->green_u_prime_day = otp_memory[0x1C];
	otp_content->green_v_prime_day = otp_memory[0x1D];
	otp_content->green_iv_day = ((uint16_t)((otp_memory[0x1E]) & 0xFF)) | ((uint16_t)(otp_memory[0x1B] & 0xF0)) << 4;

	return aoresult_ok;
}


