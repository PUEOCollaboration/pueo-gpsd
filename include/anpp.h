/******************************************************************************/
/*                                                                            */
/*                Advanced Navigation Packet Protocol Library                 */
/*                 C Language Static Boreas SDK, Version 7.3                  */
/*                    Copyright 2024, Advanced Navigation                     */
/*                                                                            */
/******************************************************************************/
/*
 * Copyright (C) 2024 Advanced Navigation
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef AN_PACKET_PROTOCOL_H_
#define AN_PACKET_PROTOCOL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define AN_PACKET_HEADER_SIZE 5
#define AN_MAXIMUM_PACKET_SIZE 255
#define AN_DECODE_BUFFER_SIZE 2*(AN_MAXIMUM_PACKET_SIZE + AN_PACKET_HEADER_SIZE)
#define AN_DECODE_MAXIMUM_FILL_SIZE (AN_MAXIMUM_PACKET_SIZE + AN_PACKET_HEADER_SIZE)
#define an_packet_pointer(packet) (packet)->header
#define an_packet_size(packet) ((packet)->length + AN_PACKET_HEADER_SIZE)*sizeof(uint8_t)

#define an_decoder_pointer(an_decoder) &(an_decoder)->buffer[(an_decoder)->buffer_length]
#define an_decoder_size(an_decoder) (sizeof((an_decoder)->buffer) - (an_decoder)->buffer_length)
#define an_decoder_increment(an_decoder, bytes_received) (an_decoder)->buffer_length += bytes_received

#ifndef FALSE
#define FALSE 0
#define TRUE 1
#endif

typedef struct
{
	uint8_t buffer[AN_DECODE_BUFFER_SIZE];
	uint16_t buffer_length;
	uint32_t crc_errors;
} an_decoder_t;

typedef struct
{
	uint8_t id;
	uint8_t length;
	uint8_t header[AN_PACKET_HEADER_SIZE];
	uint8_t data[AN_MAXIMUM_PACKET_SIZE];
} an_packet_t;

void an_decoder_initialise(an_decoder_t* an_decoder);
uint8_t an_packet_decode(struct gps_device_t *session, an_decoder_t* an_decoder, an_packet_t* an_packet);
void an_packet_encode(an_packet_t* an_packet);
uint16_t anpp_calculate_crc16(const void* data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
