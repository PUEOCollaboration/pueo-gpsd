/*
 * A prototype driver.  Doesn't run, doesn't even compile.
 *
 * For new driver authors: replace "_PROTO_" and "_proto_" with the name of
 * your new driver. That will give you a skeleton with all the required
 * functions defined.
 *
 * Once that is done, you will likely have to define a large number of
 * flags and masks. From there, you will be able to start extracting
 * useful quantities. There are roughed-in decoders for the navigation
 * solution, satellite status and gps-utc offset. These are the 3 key
 * messages that gpsd needs. Some protocols transmit error estimates
 * separately from the navigation solution; if developing a driver for
 * such a protocol you will need to add a decoder function for that
 * message. Be extra careful when using sizeof(<type>) to extract part
 * of packets (ie. don't do it). This idiom creates portability problems
 * between 32 and 64 bit systems.
 *
 * For anyone hacking this driver skeleton: "_PROTO_" and "_proto_" are now
 * reserved tokens. We suggest that they only ever be used as prefixes,
 * but if they are used infix, they must be used in a way that allows a
 * driver author to find-and-replace to create a unique namespace for
 * driver functions.
 *
 * If using vi, ":%s/_PROTO_/MYDRIVER/g" and ":%s/_proto_/mydriver/g"
 * should produce a source file that comes very close to being useful.
 * You will also need to add hooks for your new driver to:
 * SConscript
 *   in: boolopts = (
 *   in: libgpsd_sources = [
 * drivers.c
 *   beneath: 'const struct gps_type_t driver_pps = {'
 *   in: static const struct gps_type_t *gpsd_driver_array[] = {
 * gpsd.h
 *   in: struct gps_lexer_t {
 * libgpsd_core.c ????
 * packet.c
 *   in: static bool nextstate(struct gps_lexer_t *lexer, unsigned char c)
 *   in: void packet_parse(struct gps_lexer_t *lexer)
 * packet_states.h
 *  above: closing #endif
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  /* must be before all includes */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#if defined(ANPP_ENABLE)

#include "../include/bits.h"

static  gps_mask_t anpp_parse_input(struct gps_device_t *);
static  gps_mask_t anpp_dispatch(struct gps_device_t *, unsigned char *,
                                    size_t );
static  gps_mask_t anpp_msg_navsol(struct gps_device_t *, unsigned char *,
                                      size_t );
static  gps_mask_t anpp_msg_utctime(struct gps_device_t *, unsigned char *,
                                       size_t );
static  gps_mask_t anpp_msg_svinfo(struct gps_device_t *, unsigned char *,
                                      size_t );
static  gps_mask_t anpp_msg_raw(struct gps_device_t *, unsigned char *,
                                   size_t );

/*
 * These methods may be called elsewhere in gpsd
 */
static  ssize_t anpp_control_send(struct gps_device_t *, char *, size_t);
static  bool anpp_probe_detect(struct gps_device_t *);
static  void anpp_event_hook(struct gps_device_t *, event_t);
static  bool anpp_set_speed(struct gps_device_t *, speed_t, char, int);
static  void anpp_set_mode(struct gps_device_t *, int);


/*
 * Advanced Navigation Packet structure
 * ----- Packet header ----
 * Header LRC -- 1 byte, u8
 * Packet ID -- 1 byte, u8
 * Packet length -- 1 byte, u8
 * CRC -- 2 bytes, u16
 * ------------------------
 * Followed by packet data of the length specified in the header
 */

/* ----------------------------- Functions from 'an_packet_protocol.c' -------------------

/*
 * Function to calculate the CRC16 of data
 * CRC16-CCITT
 * Initial value = 0xFFFF
 * Polynomial = x^16 + x^12 + x^5 + x^0
 */
uint16_t calculate_crc16(const void* data, uint16_t length)
{
	uint8_t* bytes = (uint8_t*) data;
	uint16_t crc = 0xFFFF, i;
	for(i = 0; i < length; i++)
	{
		crc = (uint16_t) ((crc << 8) ^ crc16_table[(crc >> 8) ^ bytes[i]]);
	}
	return crc;
}

/*
 * Special version of CRC16-CCITT that does not require the CRC16 table.
 * Uses much less memory at the cost of more CPU processing time.
 * Good for devices with very limited memory.
 */
//uint16_t calculate_crc16(uint8_t* bytes, uint16_t length)
//{
//	uint16_t crc = 0xFFFF;
//
//	for(int i = 0; i < length; i++)
//	{
//		crc ^= bytes[i] << 8;
//		for(int j = 7; j >= 0; j--)
//		{
//			if(crc & 0x8000)
//			{
//				crc = (crc << 1) ^ 0x1021;
//			}
//			else
//			{
//				crc <<= 1;
//			}
//		}
//	}
//	return crc;
//}

/*
 * Function to calculate a 4 byte LRC
 */
uint8_t calculate_header_lrc(uint8_t* data)
{
	return ((data[0] + data[1] + data[2] + data[3]) ^ 0xFF) + 1;
}

/*
 * Initialise the decoder
 */
void an_decoder_initialise(an_decoder_t* an_decoder)
{
	an_decoder->buffer_length = 0;
	an_decoder->crc_errors = 0;
}

bool valid_anpp_lrc(const char *buffer)
{
  uint8_t header_lrc;
  
  header_lrc = an_decoder->buffer[0];
  return (header_lrc == calculate_header_lrc(&buffer[1]));

}

/*
 * Function to decode an_packets from raw data
 * returns TRUE (1) if a packet was decoded or FALSE (0) if no packet was decoded
 */
uint8_t an_packet_decode(an_decoder_t* an_decoder, an_packet_t* an_packet)
{
	uint16_t decode_iterator = 0;
	uint8_t packet_decoded = FALSE;
	uint8_t header_lrc;
	uint16_t crc;

	while(decode_iterator + AN_PACKET_HEADER_SIZE <= an_decoder->buffer_length)
	{
		header_lrc = an_decoder->buffer[decode_iterator++];
		if(header_lrc == calculate_header_lrc(&an_decoder->buffer[decode_iterator]))
		{
			an_packet->id = an_decoder->buffer[decode_iterator++];
			an_packet->length = an_decoder->buffer[decode_iterator++];
			crc = an_decoder->buffer[decode_iterator++];
			crc |= an_decoder->buffer[decode_iterator++] << 8;

			if(decode_iterator + an_packet->length > an_decoder->buffer_length)
			{
				decode_iterator -= AN_PACKET_HEADER_SIZE;
				break;
			}

			if(crc == calculate_crc16(&an_decoder->buffer[decode_iterator], an_packet->length))
			{
				packet_decoded = TRUE;
				memcpy(an_packet->header, &an_decoder->buffer[decode_iterator - AN_PACKET_HEADER_SIZE], AN_PACKET_HEADER_SIZE * sizeof(uint8_t));
				memcpy(an_packet->data, &an_decoder->buffer[decode_iterator], an_packet->length * sizeof(uint8_t));
				decode_iterator += an_packet->length;
				break;
			}
			else
			{
				decode_iterator -= (AN_PACKET_HEADER_SIZE - 1);
				an_decoder->crc_errors++;
			}
		}
	}
	if(decode_iterator < an_decoder->buffer_length)
	{
		if(decode_iterator > 0)
		{
			memmove(&an_decoder->buffer[0], &an_decoder->buffer[decode_iterator], (an_decoder->buffer_length - decode_iterator) * sizeof(uint8_t));
			an_decoder->buffer_length -= decode_iterator;
		}
	}
	else an_decoder->buffer_length = 0;

	return packet_decoded;
}

/*
 * Function to encode an an_packet
 */
void an_packet_encode(an_packet_t* an_packet)
{
	uint16_t crc;
	an_packet->header[1] = an_packet->id;
	an_packet->header[2] = an_packet->length;
	crc = calculate_crc16(an_packet->data, an_packet->length);
	memcpy(&an_packet->header[3], &crc, sizeof(uint16_t));
	an_packet->header[0] = calculate_header_lrc(&an_packet->header[1]);
}

/*
 * ------------------------ Functions from 'ins_packets.c' -------------------
 *
 * Decode functions take an an_packet_t and turn it into a type specific
 * to that packet so that the fields can be conveniently accessed. Decode
 * functions return 0 for success and 1 for failure. Decode functions are
 * used when receiving packets.
 *
 * Example decode
 *
 * an_packet_t an_packet
 * acknowledge_packet_t acknowledge_packet
 * ...
 * decode_acknowledge_packet(&acknowledge_packet, &an_packet);
 * printf("acknowledge id %d with result %d\n", acknowledge_packet.packet_id, acknowledge_packet.acknowledge_result);
 *
 * Encode functions take a type specific structure and turn it into an
 * an_packet_t. Encode functions are used when sending packets.
 *
 * Example encode
 *
 * an_packet_t an_packet;
 * boot_mode_packet_t boot_mode_packet;
 * ...
 * boot_mode_packet.boot_mode = boot_mode_bootloader;
 * encode_boot_mode_packet(&an_packet, &boot_mode_packet);
 * serial_port_transmit(an_packet_pointer(&an_packet), an_packet_size(&an_packet));
 *
 */
int decode_acknowledge_packet(acknowledge_packet_t* acknowledge_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_acknowledge && an_packet->length == 4)
	{
		acknowledge_packet->packet_id = an_packet->data[0];
		memcpy(&acknowledge_packet->packet_crc, &an_packet->data[1], sizeof(uint16_t));
		acknowledge_packet->acknowledge_result = an_packet->data[3];
		return 0;
	}
	else return 1;
}

void encode_request_packet(an_packet_t* an_packet, uint8_t requested_packet_id)
{
    an_packet->id = packet_id_request;
	an_packet->length = 1;
	an_packet->data[0] = requested_packet_id;
}

int decode_boot_mode_packet(boot_mode_packet_t* boot_mode_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_boot_mode && an_packet->length == 1)
	{
		boot_mode_packet->boot_mode = an_packet->data[0];
		return 0;
	}
	else return 1;
}

void encode_boot_mode_packet(an_packet_t* an_packet, boot_mode_packet_t* boot_mode_packet)
{
    an_packet->id = packet_id_boot_mode;
	an_packet->length = 1;
	an_packet->data[0] = boot_mode_packet->boot_mode;
}

int decode_device_information_packet(device_information_packet_t* device_information_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_device_information && an_packet->length == 24)
	{
		memcpy(&device_information_packet->software_version, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&device_information_packet->device_id, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&device_information_packet->hardware_revision, &an_packet->data[8], sizeof(uint32_t));
		memcpy(&device_information_packet->serial_number[0], &an_packet->data[12], 3 * sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

void encode_restore_factory_settings_packet(an_packet_t* an_packet)
{
	uint32_t verification = 0x85429E1C;
    an_packet->id = packet_id_restore_factory_settings;
	an_packet->length = 4;
	memcpy(&an_packet->data[0], &verification, sizeof(uint32_t));
}

void encode_reset_packet(an_packet_t* an_packet)
{
	uint32_t verification = 0x21057A7E;
    an_packet->id = packet_id_reset;
	an_packet->length = 4;
	memcpy(&an_packet->data[0], &verification, sizeof(uint32_t));
}

int decode_file_transfer_acknowledge_packet(file_transfer_acknowledge_packet_t* file_transfer_acknowledge_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_file_transfer_acknowledge && an_packet->length == 9)
	{
		memcpy(&file_transfer_acknowledge_packet->unique_id, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&file_transfer_acknowledge_packet->data_index, &an_packet->data[4], sizeof(uint32_t));
		file_transfer_acknowledge_packet->response_code = an_packet->data[8];
		return 0;
	}
	else return 1;
}

void encode_file_transfer_packet(an_packet_t* an_packet, file_transfer_packet_t* file_transfer_packet, int data_size)
{
    an_packet->id = packet_id_file_transfer;
	if(file_transfer_packet != NULL)
	{
		if(file_transfer_packet->data_index == 0)
		{
			an_packet->length = 3 * sizeof(uint32_t) + 2 * sizeof(uint8_t) + file_transfer_packet->metadata_length + data_size;
			memcpy(&an_packet->data[0], &file_transfer_packet->unique_id, sizeof(uint32_t));
			memcpy(&an_packet->data[4], &file_transfer_packet->data_index, sizeof(uint32_t));
			memcpy(&an_packet->data[8], &file_transfer_packet->total_size, sizeof(uint32_t));
			an_packet->data[12] = file_transfer_packet->data_encoding;
			an_packet->data[13] = file_transfer_packet->metadata_type;
			if(file_transfer_packet->metadata_type != file_transfer_metadata_none)
			{
				an_packet->length = an_packet->length + sizeof(uint16_t);
				memcpy(&an_packet->data[14], &file_transfer_packet->metadata_length, sizeof(uint16_t));
				memcpy(&an_packet->data[16], file_transfer_packet->metadata, file_transfer_packet->metadata_length);
				memcpy(&an_packet->data[16+file_transfer_packet->metadata_length], file_transfer_packet->packet_data, data_size);
			}
			else
			{
				memcpy(&an_packet->data[14], file_transfer_packet->packet_data, data_size);
			}
		}
		else
		{
			an_packet->length = 2 * sizeof(uint32_t) + data_size;
			memcpy(&an_packet->data[0], &file_transfer_packet->unique_id, sizeof(uint32_t));
			memcpy(&an_packet->data[4], &file_transfer_packet->data_index, sizeof(uint32_t));
			memcpy(&an_packet->data[8], file_transfer_packet->packet_data, data_size);
		}
	}
}

int decode_serial_port_passthrough_packet(serial_port_passthrough_packet_t* serial_port_passthrough_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_serial_port_passthrough && an_packet->length <= 255)
	{
		serial_port_passthrough_packet->passthrough_route = an_packet->data[0];
		memcpy(&serial_port_passthrough_packet->passthrough_data, &an_packet->data[1], an_packet->length - sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

void encode_serial_port_passthrough_packet(an_packet_t* an_packet, serial_port_passthrough_packet_t* serial_port_passthrough_packet, int data_size)
{
    an_packet->id = packet_id_serial_port_passthrough;
	an_packet->length = sizeof(uint8_t) + data_size;
	an_packet->data[0] = serial_port_passthrough_packet->passthrough_route;
	memcpy(&an_packet->data[1], &serial_port_passthrough_packet->passthrough_data, data_size);
}

int decode_ip_configuration_packet(ip_configuration_packet_t* ip_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_ip_configuration && an_packet->length == 30)
	{
		ip_configuration_packet->permanent = an_packet->data[0];
		memcpy(&ip_configuration_packet->dhcp_mode, &an_packet->data[1], sizeof(uint8_t));
		memcpy(&ip_configuration_packet->ip_address, &an_packet->data[2], sizeof(uint32_t));
		memcpy(&ip_configuration_packet->ip_netmask, &an_packet->data[6], sizeof(uint32_t));
		memcpy(&ip_configuration_packet->ip_gateway, &an_packet->data[10], sizeof(uint32_t));
		memcpy(&ip_configuration_packet->dns_server, &an_packet->data[14], sizeof(uint32_t));
		memcpy(&ip_configuration_packet->serial_number[0], &an_packet->data[18], 3 * sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

void encode_ip_configuration_packet(an_packet_t* an_packet, ip_configuration_packet_t* ip_configuration_packet)
{
    an_packet->id = packet_id_ip_configuration;
	an_packet->length = 30;
	an_packet->data[0] = ip_configuration_packet->permanent;
	memcpy(&an_packet->data[1], &ip_configuration_packet->dhcp_mode, sizeof(uint8_t));
	memcpy(&an_packet->data[2], &ip_configuration_packet->ip_address, sizeof(uint32_t));
	memcpy(&an_packet->data[6], &ip_configuration_packet->ip_netmask, sizeof(uint32_t));
	memcpy(&an_packet->data[10], &ip_configuration_packet->ip_gateway, sizeof(uint32_t));
	memcpy(&an_packet->data[14], &ip_configuration_packet->dns_server, sizeof(uint32_t));
	memcpy(&an_packet->data[18], &ip_configuration_packet->serial_number[0], 3 * sizeof(uint32_t));
}

int decode_extended_device_information_packet(extended_device_information_packet_t* extended_device_information_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_extended_device_information && an_packet->length == 36)
	{
		memcpy(&extended_device_information_packet->software_version, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&extended_device_information_packet->device_id, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&extended_device_information_packet->hardware_revision, &an_packet->data[8], sizeof(uint32_t));
		memcpy(&extended_device_information_packet->serial_number[0], &an_packet->data[12], 3 * sizeof(uint32_t));
		memcpy(&extended_device_information_packet->device_subtype, &an_packet->data[16], sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

int decode_subcomponent_information_packet(subcomponent_information_packet_t* subcomponent_information_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_subcomponent_information && an_packet->length % 24 == 0)
	{
		int number_of_subcomponents = an_packet->length / 24;
		for(int i = 0; i < MAXIMUM_SUBCOMPONENTS; i++)
		{
			if (i < number_of_subcomponents)
			{
				memcpy(&subcomponent_information_packet->subcomponent_information[i].software_version, &an_packet->data[24 * i], sizeof(uint32_t));
				memcpy(&subcomponent_information_packet->subcomponent_information[i].device_id, &an_packet->data[24 * i + 4], sizeof(uint32_t));
				memcpy(&subcomponent_information_packet->subcomponent_information[i].hardware_revision, &an_packet->data[24 * i + 8], sizeof(uint32_t));
				memcpy(&subcomponent_information_packet->subcomponent_information[i].serial_number[0], &an_packet->data[24 * i + 12], 3 * sizeof(uint32_t));
			}
			else memset(&subcomponent_information_packet->subcomponent_information[i], 0, sizeof(subcomponent_information_t));
		}
		return 0;
	}
	else return 1;
}

int decode_system_state_packet(system_state_packet_t* system_state_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_system_state && an_packet->length == 100)
	{
		memcpy(&system_state_packet->system_status, &an_packet->data[0], sizeof(uint16_t));
		memcpy(&system_state_packet->filter_status, &an_packet->data[2], sizeof(uint16_t));
		memcpy(&system_state_packet->unix_time_seconds, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&system_state_packet->microseconds, &an_packet->data[8], sizeof(uint32_t));
		memcpy(&system_state_packet->latitude, &an_packet->data[12], sizeof(double));
		memcpy(&system_state_packet->longitude, &an_packet->data[20], sizeof(double));
		memcpy(&system_state_packet->height, &an_packet->data[28], sizeof(double));
		memcpy(&system_state_packet->velocity[0], &an_packet->data[36], 3 * sizeof(float));
		memcpy(&system_state_packet->body_acceleration[0], &an_packet->data[48], 3 * sizeof(float));
		memcpy(&system_state_packet->g_force, &an_packet->data[60], sizeof(float));
		memcpy(&system_state_packet->orientation[0], &an_packet->data[64], 3 * sizeof(float));
		memcpy(&system_state_packet->angular_velocity[0], &an_packet->data[76], 3 * sizeof(float));
		memcpy(&system_state_packet->standard_deviation[0], &an_packet->data[88], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}


// ----------------- Unix time ----------------------- //
int decode_unix_time_packet(unix_time_packet_t* unix_time_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_unix_time && an_packet->length == 8)
	{
		memcpy(&unix_time_packet->unix_time_seconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&unix_time_packet->microseconds, &an_packet->data[4], sizeof(uint32_t));
		return 0;
	}
	else return 1;
}
static gps_mask_t anpp_unix_time(struct gps_device_t *session, an_packet_t* an_packet) {
  
  gps_mask_t mask = 0;

  unix_time_packet_t unix_time_packet;
  
  if (decode_unix_time_packet(&unix_time_packet, an_packet) == 0) {
    double T = system_state_packet.unix_time_seconds + (system_state_packet.microseconds/1000000.0);    

    // TODO: Merge this into gpsd, but need to convert to GPS time first
    GPSD_LOG(LOG_PROG, &session->context->errout,
	     "ANPP: Unix time: %.5f\n",
	     T);
  }
  else {
    GPSD_LOG(LOG_WARN, &session->context->errout,
	     "ANPP: Unix time: unable to decode");
    return 0;
  }
  
  return mask;  
}
// ------------------------------------------------------ //

int decode_formatted_time_packet(formatted_time_packet_t* formatted_time_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_formatted_time && an_packet->length == 14)
	{
		memcpy(&formatted_time_packet->microseconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&formatted_time_packet->year, &an_packet->data[4], sizeof(uint16_t));
		memcpy(&formatted_time_packet->year_day, &an_packet->data[6], sizeof(uint16_t));
		formatted_time_packet->month = an_packet->data[8];
		formatted_time_packet->month_day = an_packet->data[9];
		formatted_time_packet->week_day = an_packet->data[10];
		formatted_time_packet->hour = an_packet->data[11];
		formatted_time_packet->minute = an_packet->data[12];
		formatted_time_packet->second = an_packet->data[13];
		return 0;
	}
	else return 1;
}

int decode_status_packet(status_packet_t* status_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_status && an_packet->length == 4)
	{
		memcpy(&status_packet->system_status, &an_packet->data[0], sizeof(uint16_t));
		memcpy(&status_packet->filter_status, &an_packet->data[2], sizeof(uint16_t));
		return 0;
	}
	else return 1;
}

int decode_position_standard_deviation_packet(position_standard_deviation_packet_t* position_standard_deviation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_position_standard_deviation && an_packet->length == 12)
	{
		memcpy(&position_standard_deviation_packet->standard_deviation[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_velocity_standard_deviation_packet(velocity_standard_deviation_packet_t* velocity_standard_deviation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_velocity_standard_deviation && an_packet->length == 12)
	{
		memcpy(&velocity_standard_deviation_packet->standard_deviation[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}


// -------------- Euler orientation STD ----------------- //
int decode_euler_orientation_standard_deviation_packet(euler_orientation_standard_deviation_packet_t* euler_orientation_standard_deviation, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_euler_orientation_standard_deviation && an_packet->length == 12)
	{
		memcpy(&euler_orientation_standard_deviation->standard_deviation[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}
static gps_mask_t anpp_euler_orientation_standard_deviation(struct gps_device_t *session, an_packet_t* an_packet) {
  gps_mask_t mask = 0;

  euler_orientation_packet_standard_deviation_t euler_orientation_standard_deviation_packet;
  // GPSD expects attitude in degrees
  
  if (decode_euler_orientation_packet(&euler_orientation_packet, an_packet) == 0) {
    session->gpsdata.attitude.roll_std = euler_orientation_packet.standard_deviation[0]*RAD_2_DEG;
    session->gpsdata.attitude.pitch_std = euler_orientation_packet.standard_deviation[1]*RAD_2_DEG;
    session->gpsdata.attitude.heading_std = euler_orientation_packet.standard_deviation[2]*RAD_2_DEG;
    //mask |= ERROR_SET; // not sure if this is correct...

    GPSD_LOG(LOG_PROG, &session->context->errout,
	     "ANPP: Euler attitude STD: roll %.5f pitch %.5f heading %.5f\n",
	     session->gpsdata.attitude.roll_std,
	     session->gpsdata.attitude.pitch_std,
	     session->gpsdata.attitude.heading_std);
  }
  else {
    GPSD_LOG(LOG_WARN, &session->context->errout,
	     "ANPP: Euler attitude STD: unable to decode");
    return 0;
  }
  
  return mask;  
}
// ------------------------------------------------------ //

int decode_quaternion_orientation_standard_deviation_packet(quaternion_orientation_standard_deviation_packet_t* quaternion_orientation_standard_deviation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_quaternion_orientation_standard_deviation && an_packet->length == 16)
	{
		memcpy(&quaternion_orientation_standard_deviation_packet->standard_deviation[0], &an_packet->data[0], 4 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_raw_sensors_packet(raw_sensors_packet_t* raw_sensors_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_raw_sensors && an_packet->length == 48)
	{
		memcpy(&raw_sensors_packet->accelerometers[0], &an_packet->data[0], 3 * sizeof(float));
		memcpy(&raw_sensors_packet->gyroscopes[0], &an_packet->data[12], 3 * sizeof(float));
		memcpy(&raw_sensors_packet->magnetometers[0], &an_packet->data[24], 3 * sizeof(float));
		memcpy(&raw_sensors_packet->imu_temperature, &an_packet->data[36], sizeof(float));
		memcpy(&raw_sensors_packet->pressure, &an_packet->data[40], sizeof(float));
		memcpy(&raw_sensors_packet->pressure_temperature, &an_packet->data[44], sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_raw_gnss_packet(raw_gnss_packet_t* raw_gnss_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_raw_gnss && an_packet->length == 74)
	{
		memcpy(&raw_gnss_packet->unix_time_seconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&raw_gnss_packet->microseconds, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&raw_gnss_packet->position[0], &an_packet->data[8], 3 * sizeof(double));
		memcpy(&raw_gnss_packet->velocity[0], &an_packet->data[32], 3 * sizeof(float));
		memcpy(&raw_gnss_packet->position_standard_deviation[0], &an_packet->data[44], 3 * sizeof(float));
		memcpy(&raw_gnss_packet->tilt, &an_packet->data[56], sizeof(float));
		memcpy(&raw_gnss_packet->heading, &an_packet->data[60], sizeof(float));
		memcpy(&raw_gnss_packet->tilt_standard_deviation, &an_packet->data[64], sizeof(float));
		memcpy(&raw_gnss_packet->heading_standard_deviation, &an_packet->data[68], sizeof(float));
		memcpy(&raw_gnss_packet->flags.r, &an_packet->data[72], sizeof(uint16_t));
		return 0;
	}
	else return 1;
}

void encode_raw_gnss_packet(an_packet_t* an_packet, raw_gnss_packet_t* raw_gnss_packet)
{
	an_packet->id = packet_id_raw_gnss;
	an_packet->length = 74;
	memcpy(&an_packet->data[0], &raw_gnss_packet->unix_time_seconds, sizeof(uint32_t));
	memcpy(&an_packet->data[4], &raw_gnss_packet->microseconds, sizeof(uint32_t));
	memcpy(&an_packet->data[8], &raw_gnss_packet->position[0], 3 * sizeof(double));
	memcpy(&an_packet->data[32], &raw_gnss_packet->velocity[0], 3 * sizeof(float));
	memcpy(&an_packet->data[44], &raw_gnss_packet->position_standard_deviation[0], 3 * sizeof(float));
	memcpy(&an_packet->data[56], &raw_gnss_packet->tilt, sizeof(float));
	memcpy(&an_packet->data[60], &raw_gnss_packet->heading, sizeof(float));
	memcpy(&an_packet->data[64], &raw_gnss_packet->tilt_standard_deviation, sizeof(float));
	memcpy(&an_packet->data[68], &raw_gnss_packet->heading_standard_deviation, sizeof(float));
	memcpy(&an_packet->data[72], &raw_gnss_packet->flags.r, sizeof(uint16_t));
}

int decode_satellites_packet(satellites_packet_t* satellites_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_satellites && an_packet->length == 13)
	{
		memcpy(&satellites_packet->hdop, &an_packet->data[0], sizeof(float));
		memcpy(&satellites_packet->vdop, &an_packet->data[4], sizeof(float));
		memcpy(&satellites_packet->gps_satellites, &an_packet->data[8], 5 * sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

int decode_geodetic_position_packet(geodetic_position_packet_t* geodetic_position_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_geodetic_position && an_packet->length == 24)
	{
		memcpy(&geodetic_position_packet->position[0], &an_packet->data[0], 3 * sizeof(double));
		return 0;
	}
	else return 1;
}

int decode_ecef_position_packet(ecef_position_packet_t* ecef_position_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_ecef_position && an_packet->length == 24)
	{
		memcpy(&ecef_position_packet->position[0], &an_packet->data[0], 3 * sizeof(double));
		return 0;
	}
	else return 1;
}

int decode_utm_position_packet(utm_position_packet_t* utm_position_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_utm_position && an_packet->length == 26)
	{
		memcpy(&utm_position_packet->position, &an_packet->data[0], 3 * sizeof(double));
		utm_position_packet->zone_number = an_packet->data[24];
		utm_position_packet->zone_char = an_packet->data[25];
		return 0;
	}
	else return 1;
}

int decode_ned_velocity_packet(ned_velocity_packet_t* ned_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_ned_velocity && an_packet->length == 12)
	{
		memcpy(&ned_velocity_packet->velocity, &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_body_velocity_packet(body_velocity_packet_t* body_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_body_velocity && an_packet->length == 12)
	{
		memcpy(&body_velocity_packet->velocity, &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_acceleration_packet(acceleration_packet_t* acceleration, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_acceleration && an_packet->length == 12)
	{
		memcpy(&acceleration->acceleration[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_body_acceleration_packet(body_acceleration_packet_t* body_acceleration, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_body_acceleration && an_packet->length == 16)
	{
		memcpy(&body_acceleration->acceleration[0], &an_packet->data[0], 3 * sizeof(float));
		memcpy(&body_acceleration->g_force, &an_packet->data[12], sizeof(float));
		return 0;
	}
	else return 1;
}


// ----------------- Euler orientation -------------------//
int decode_euler_orientation_packet(euler_orientation_packet_t* euler_orientation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_euler_orientation && an_packet->length == 12)
	{
		memcpy(&euler_orientation_packet->orientation[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

static gps_mask_t anpp_euler_orientation(struct gps_device_t *session, an_packet_t* an_packet) {
  gps_mask_t mask = 0;

  euler_orientation_packet_t euler_orientation_packet;
  // Decode the packet
  // GPSD expects attitude in degrees
  
  if (decode_euler_orientation_packet(&euler_orientation_packet, an_packet) == 0) {
    session->gpsdata.attitude.roll = euler_orientation_packet.orientation[0]*RAD_2_DEG;
    session->gpsdata.attitude.pitch = euler_orientation_packet.orientation[1]*RAD_2_DEG;
    session->gpsdata.attitude.heading = euler_orientation_packet.orientation[2]*RAD_2_DEG;
    mask |= ATTITUDE_SET;

    GPSD_LOG(LOG_PROG, &session->context->errout,
	     "ANPP: Euler attitude: roll %.5f pitch %.5f heading %.5f\n",
	     session->gpsdata.attitude.roll,
	     session->gpsdata.attitude.pitch,
	     session->gpsdata.attitude.heading);
  }
  else {
    GPSD_LOG(LOG_WARN, &session->context->errout,
	     "ANPP: Euler attitude: unable to decode");
    return 0;
  }
  
  return mask;  
}
// ----------------------------------------------------------//


int decode_quaternion_orientation_packet(quaternion_orientation_packet_t* quaternion_orientation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_quaternion_orientation && an_packet->length == 16)
	{
		memcpy(&quaternion_orientation_packet->orientation[0], &an_packet->data[0], 4 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_dcm_orientation_packet(dcm_orientation_packet_t* dcm_orientation_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_dcm_orientation && an_packet->length == 36)
	{
		memcpy(&dcm_orientation_packet->orientation[0][0], &an_packet->data[0], 9 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_angular_velocity_packet(angular_velocity_packet_t* angular_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_angular_velocity && an_packet->length == 12)
	{
		memcpy(&angular_velocity_packet->angular_velocity[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_angular_acceleration_packet(angular_acceleration_packet_t* angular_acceleration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_angular_acceleration && an_packet->length == 12)
	{
		memcpy(&angular_acceleration_packet->angular_acceleration[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_external_position_velocity_packet(external_position_velocity_packet_t* external_position_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_position_velocity && an_packet->length == 60)
	{
		memcpy(&external_position_velocity_packet->position[0], &an_packet->data[0], 3 * sizeof(double));
		memcpy(&external_position_velocity_packet->velocity[0], &an_packet->data[24], 3 * sizeof(float));
		memcpy(&external_position_velocity_packet->position_standard_deviation[0], &an_packet->data[36], 3 * sizeof(float));
		memcpy(&external_position_velocity_packet->velocity_standard_deviation[0], &an_packet->data[48], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_position_velocity_packet(an_packet_t* an_packet, external_position_velocity_packet_t* external_position_velocity_packet)
{
    an_packet->id = packet_id_external_position_velocity;
	an_packet->length = 60;
	memcpy(&an_packet->data[0], &external_position_velocity_packet->position[0], 3 * sizeof(double));
	memcpy(&an_packet->data[24], &external_position_velocity_packet->velocity[0], 3 * sizeof(float));
	memcpy(&an_packet->data[36], &external_position_velocity_packet->position_standard_deviation[0], 3 * sizeof(float));
	memcpy(&an_packet->data[48], &external_position_velocity_packet->velocity_standard_deviation[0], 3 * sizeof(float));
}

int decode_external_position_packet(external_position_packet_t* external_position_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_position && an_packet->length == 36)
	{
		memcpy(&external_position_packet->position[0], &an_packet->data[0], 3 * sizeof(double));
		memcpy(&external_position_packet->standard_deviation[0], &an_packet->data[24], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_position_packet(an_packet_t* an_packet, external_position_packet_t* external_position_packet)
{
    an_packet->id = packet_id_external_position;
	an_packet->length = 36;
	memcpy(&an_packet->data[0], &external_position_packet->position[0], 3 * sizeof(double));
	memcpy(&an_packet->data[24], &external_position_packet->standard_deviation[0], 3 * sizeof(float));
}

int decode_external_velocity_packet(external_velocity_packet_t* external_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_velocity && an_packet->length == 24)
	{
		memcpy(&external_velocity_packet->velocity[0], &an_packet->data[0], 3 * sizeof(float));
		memcpy(&external_velocity_packet->standard_deviation[0], &an_packet->data[12], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_velocity_packet(an_packet_t* an_packet, external_velocity_packet_t* external_velocity_packet)
{
    an_packet->id = packet_id_external_velocity;
	an_packet->length = 24;
	memcpy(&an_packet->data[0], &external_velocity_packet->velocity[0], 3 * sizeof(float));
	memcpy(&an_packet->data[12], &external_velocity_packet->standard_deviation[0], 3 * sizeof(float));
}

int decode_external_body_velocity_packet(external_body_velocity_packet_t* external_body_velocity_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_body_velocity && an_packet->length == 16)
	{
		memcpy(&external_body_velocity_packet->velocity, &an_packet->data[0], 3 * sizeof(float));
		memcpy(&external_body_velocity_packet->standard_deviation, &an_packet->data[12], sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_body_velocity_packet(an_packet_t* an_packet, external_body_velocity_packet_t* external_body_velocity_packet)
{
    an_packet->id = packet_id_external_body_velocity;
	an_packet->length = 16;
	memcpy(&an_packet->data[0], &external_body_velocity_packet->velocity[0], 3 * sizeof(float));
	memcpy(&an_packet->data[12], &external_body_velocity_packet->standard_deviation, sizeof(float));
}

int decode_external_heading_packet(external_heading_packet_t* external_heading_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_heading && an_packet->length == 8)
	{
		memcpy(&external_heading_packet->heading, &an_packet->data[0], sizeof(float));
		memcpy(&external_heading_packet->standard_deviation, &an_packet->data[4], sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_heading_packet(an_packet_t* an_packet, external_heading_packet_t* external_heading_packet)
{
    an_packet->id = packet_id_external_heading;
	an_packet->length = 8;
	memcpy(&an_packet->data[0], &external_heading_packet->heading, sizeof(float));
	memcpy(&an_packet->data[4], &external_heading_packet->standard_deviation, sizeof(float));
}

int decode_running_time_packet(running_time_packet_t* running_time_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_running_time && an_packet->length == 8)
	{
		memcpy(&running_time_packet->seconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&running_time_packet->microseconds, &an_packet->data[4], sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

int decode_local_magnetics_packet(local_magnetics_packet_t* local_magnetics_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_local_magnetics && an_packet->length == 12)
	{
		memcpy(&local_magnetics_packet->magnetic_field[0], &an_packet->data[0], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_odometer_state_packet(odometer_state_packet_t* odometer_state_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_odometer_state && an_packet->length == 20)
	{
		memcpy(&odometer_state_packet->pulse_count, &an_packet->data[0], sizeof(int32_t));
		memcpy(&odometer_state_packet->distance, &an_packet->data[4], sizeof(float));
		memcpy(&odometer_state_packet->speed, &an_packet->data[8], sizeof(float));
		memcpy(&odometer_state_packet->slip, &an_packet->data[12], sizeof(float));
		odometer_state_packet->active = an_packet->data[16];
		return 0;
	}
	else return 1;
}

int decode_external_time_packet(external_time_packet_t* external_time_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_time && an_packet->length == 8)
	{
		memcpy(&external_time_packet->unix_time_seconds, &an_packet->data[0], sizeof(float));
		memcpy(&external_time_packet->microseconds, &an_packet->data[4], sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_time_packet(an_packet_t* an_packet, external_time_packet_t* external_time_packet)
{
    an_packet->id = packet_id_external_time;
	an_packet->length = 8;
	memcpy(&an_packet->data[0], &external_time_packet->unix_time_seconds, sizeof(float));
	memcpy(&an_packet->data[4], &external_time_packet->microseconds, sizeof(float));
}

int decode_external_depth_packet(external_depth_packet_t* external_depth_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_depth && an_packet->length == 8)
	{
		memcpy(&external_depth_packet->depth, &an_packet->data[0], sizeof(float));
		memcpy(&external_depth_packet->standard_deviation, &an_packet->data[4], sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_depth_packet(an_packet_t* an_packet, external_depth_packet_t* external_depth_packet)
{
    an_packet->id = packet_id_external_depth;
	an_packet->length = 8;
	memcpy(&an_packet->data[0], &external_depth_packet->depth, sizeof(float));
	memcpy(&an_packet->data[4], &external_depth_packet->standard_deviation, sizeof(float));
}

int decode_geoid_height_packet(geoid_height_packet_t* geoid_height_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_geoid_height && an_packet->length == 4)
	{
		memcpy(&geoid_height_packet->geoid_height, &an_packet->data[0], sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_rtcm_corrections_packet(an_packet_t* an_packet, rtcm_corrections_packet_t* rtcm_corrections_packet, uint8_t data_size)
{
    an_packet->id = packet_id_rtcm_corrections;
	an_packet->length = data_size;
	memcpy(&an_packet->data[0], rtcm_corrections_packet->packet_data, data_size);
}

int decode_wind_packet(wind_packet_t* wind_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_wind && an_packet->length == 12)
	{
		memcpy(&wind_packet->wind_velocity[0], &an_packet->data[0], 2 * sizeof(float));
		memcpy(&wind_packet->wind_standard_deviation, &an_packet->data[8], sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_wind_packet(an_packet_t* an_packet, wind_packet_t* wind_packet)
{
    an_packet->id = packet_id_wind;
	an_packet->length = 12;
	memcpy(&an_packet->data[0], &wind_packet->wind_velocity[0], 2 * sizeof(float));
	memcpy(&an_packet->data[8], &wind_packet->wind_standard_deviation, sizeof(float));
}

int decode_heave_packet(heave_packet_t* heave_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_heave && an_packet->length == 16)
	{
		memcpy(&heave_packet->heave_point_1, &an_packet->data[0], sizeof(float));
		memcpy(&heave_packet->heave_point_2, &an_packet->data[4], sizeof(float));
		memcpy(&heave_packet->heave_point_3, &an_packet->data[8], sizeof(float));
		memcpy(&heave_packet->heave_point_4, &an_packet->data[12], sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_raw_satellite_data_packet(raw_satellite_data_packet_t* raw_satellite_data_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_raw_satellite_data && an_packet->length >= 16)
	{
		memcpy(&raw_satellite_data_packet->unix_time_seconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&raw_satellite_data_packet->nanoseconds, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&raw_satellite_data_packet->receiver_clock_offset, &an_packet->data[8], sizeof(int32_t));
		raw_satellite_data_packet->receiver_number = an_packet->data[12];
		raw_satellite_data_packet->packet_number = an_packet->data[13];
		raw_satellite_data_packet->total_packets = an_packet->data[14];
		raw_satellite_data_packet->number_of_satellites = an_packet->data[15];
		int index = 16;
		for(int i = 0; i < raw_satellite_data_packet->number_of_satellites; i++)
		{
			raw_satellite_data_packet->satellite[i].satellite_system = an_packet->data[index];
			raw_satellite_data_packet->satellite[i].prn = an_packet->data[index + 1];
			raw_satellite_data_packet->satellite[i].elevation = an_packet->data[index + 2];
			memcpy(&raw_satellite_data_packet->satellite[i].azimuth, &an_packet->data[index + 3], sizeof(uint16_t));
			raw_satellite_data_packet->satellite[i].number_of_frequencies = an_packet->data[index + 5];
			index += 6;
			for(int j = 0; j < raw_satellite_data_packet->satellite[i].number_of_frequencies; j++)
			{
				raw_satellite_data_packet->satellite[i].frequency[j].satellite_frequency = an_packet->data[index];
				memcpy(&raw_satellite_data_packet->satellite[i].frequency[j].tracking_status, &an_packet->data[index + 1], sizeof(uint8_t));
				memcpy(&raw_satellite_data_packet->satellite[i].frequency[j].carrier_phase, &an_packet->data[index + 2], sizeof(double));
				memcpy(&raw_satellite_data_packet->satellite[i].frequency[j].pseudo_range, &an_packet->data[index + 10], sizeof(double));
				memcpy(&raw_satellite_data_packet->satellite[i].frequency[j].doppler_frequency, &an_packet->data[index + 18], sizeof(float));
				memcpy(&raw_satellite_data_packet->satellite[i].frequency[j].signal_to_noise_ratio, &an_packet->data[index + 22], sizeof(float));
				index += 26;
			}
		}
		return 0;
	}
	else return 1;
}

int decode_raw_satellite_ephemeris_packet(raw_satellite_ephemeris_packet_t* raw_satellite_ephemeris_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_raw_satellite_ephemeris)
	{
		uint8_t satellite_system = an_packet->data[4];
		if(an_packet->length == 132 && satellite_system == satellite_system_gps)
		{
			memcpy(&raw_satellite_ephemeris_packet->gps.unix_time, &an_packet->data[0], sizeof(uint32_t));
			raw_satellite_ephemeris_packet->gps.satellite_system = an_packet->data[4];
			raw_satellite_ephemeris_packet->gps.satellite_number = an_packet->data[5];
			memcpy(&raw_satellite_ephemeris_packet->gps.time_of_ephemeris, &an_packet->data[6], sizeof(uint32_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.issue_of_data_clock, &an_packet->data[10], sizeof(uint16_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.issue_of_data_ephemeris, &an_packet->data[12], sizeof(uint16_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.satellite_clock_bias, &an_packet->data[14], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.satellite_clock_drift, &an_packet->data[18], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.satellite_clock_drift_rate, &an_packet->data[22], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.crs, &an_packet->data[26], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.deltaN, &an_packet->data[30], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.m0, &an_packet->data[34], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.cuc, &an_packet->data[42], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.eccentricity, &an_packet->data[46], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.cus, &an_packet->data[54], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.sqrtA, &an_packet->data[58], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.cic, &an_packet->data[66], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.omega0, &an_packet->data[70], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.cis, &an_packet->data[78], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.i0, &an_packet->data[82], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.crc, &an_packet->data[90], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.omega, &an_packet->data[94], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.omega_dot, &an_packet->data[102], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.i_dot, &an_packet->data[110], sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps.tgd, &an_packet->data[118], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->gps.ephemeris_week_number, &an_packet->data[122], sizeof(uint16_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.transmission_time, &an_packet->data[124], sizeof(uint32_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.user_range_accuracy, &an_packet->data[128], sizeof(uint16_t));
			memcpy(&raw_satellite_ephemeris_packet->gps.flags, &an_packet->data[130], sizeof(uint16_t));
			return 0;
		}
		else if(an_packet->length == 94 && satellite_system == satellite_system_glonass)
		{
			memcpy(&raw_satellite_ephemeris_packet->glo.unix_time, &an_packet->data[0], sizeof(uint32_t));
			raw_satellite_ephemeris_packet->glo.satellite_system = an_packet->data[4];
			raw_satellite_ephemeris_packet->glo.satellite_number = an_packet->data[5];
			memcpy(&raw_satellite_ephemeris_packet->glo.clock_bias, &an_packet->data[6], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->glo.frequency_bias, &an_packet->data[10], sizeof(float));
			memcpy(&raw_satellite_ephemeris_packet->glo.position[0], &an_packet->data[14], 3 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->glo.velocity[0], &an_packet->data[38], 3 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->glo.acceleration[0], &an_packet->data[62], 3 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->glo.frame_start_time, &an_packet->data[86], sizeof(uint32_t));
			raw_satellite_ephemeris_packet->glo.age = an_packet->data[90];
			raw_satellite_ephemeris_packet->glo.frequency_slot = an_packet->data[91];
			raw_satellite_ephemeris_packet->glo.satellite_health = an_packet->data[92];
			return 0;
		}
		else if(an_packet->length == 92 && satellite_system == satellite_system_gps)
		{
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.unix_time, &an_packet->data[0], sizeof(uint32_t));
			raw_satellite_ephemeris_packet->gps_iono.satellite_system = an_packet->data[4];
			raw_satellite_ephemeris_packet->gps_iono.satellite_number = an_packet->data[5];
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.alpha[0], &an_packet->data[6], 4 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.beta[0], &an_packet->data[38], 4 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.asub[0], &an_packet->data[70], 2 * sizeof(double));
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.delta_ls, &an_packet->data[86], sizeof(int16_t));
			memcpy(&raw_satellite_ephemeris_packet->gps_iono.delta_tlsf, &an_packet->data[88], sizeof(int16_t));
			raw_satellite_ephemeris_packet->gps_iono.wnsub_lsf = an_packet->data[90];
			raw_satellite_ephemeris_packet->gps_iono.dn = an_packet->data[91];
			return 0;
		}
		else return 1;
	}
	else return 1;
}

int decode_gnss_summary_packet(gnss_summary_packet_t* gnss_summary_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gnss_summary && an_packet->length == 40)
	{
		memcpy(&gnss_summary_packet->geoid_height, &an_packet->data[0], sizeof(float));
		memcpy(&gnss_summary_packet->pdop, &an_packet->data[4], sizeof(float));
		memcpy(&gnss_summary_packet->hdop, &an_packet->data[8], sizeof(float));
		memcpy(&gnss_summary_packet->vdop, &an_packet->data[12], sizeof(float));
		memcpy(&gnss_summary_packet->tdop, &an_packet->data[16], sizeof(float));
		memcpy(&gnss_summary_packet->gps_utc_offset, &an_packet->data[20], sizeof(int32_t));
		memcpy(&gnss_summary_packet->heading, &an_packet->data[24], sizeof(float));
		gnss_summary_packet->gps_satellites = an_packet->data[29];
		gnss_summary_packet->glonass_satellites = an_packet->data[30];
		gnss_summary_packet->sbas_satellites = an_packet->data[31];
		gnss_summary_packet->beidou_satellites = an_packet->data[32];
		gnss_summary_packet->galileo_satellites = an_packet->data[33];
		memset(&an_packet->data[34], 0, 6 * sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

int decode_external_odometer_packet(odometer_packet_t* external_odometer_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_odometer && an_packet->length == 13)
	{
		memcpy(&external_odometer_packet->delay, &an_packet->data[0], sizeof(float));
		memcpy(&external_odometer_packet->speed, &an_packet->data[4], sizeof(float));
		memcpy(&external_odometer_packet->distance_travelled, &an_packet->data[8], sizeof(float));
		external_odometer_packet->flags.r = an_packet->data[12];
		return 0;
	}
	else return 1;
}

void encode_external_odometer_packet(an_packet_t* an_packet, odometer_packet_t* external_odometer_packet)
{
    an_packet->id = packet_id_external_odometer;
	an_packet->length = 13;
	memcpy(&an_packet->data[0], &external_odometer_packet->delay, sizeof(float));
	memcpy(&an_packet->data[4], &external_odometer_packet->speed, sizeof(float));
	memcpy(&an_packet->data[8], &external_odometer_packet->distance_travelled, sizeof(float));
	an_packet->data[12] = external_odometer_packet->flags.r;
}

int decode_external_air_data_packet(external_air_data_packet_t* external_air_data_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_air_data && an_packet->length == 25)
	{
		memcpy(&external_air_data_packet->barometric_altitude_delay, &an_packet->data[0], sizeof(float));
		memcpy(&external_air_data_packet->airspeed_delay, &an_packet->data[4], sizeof(float));
		memcpy(&external_air_data_packet->barometric_altitude, &an_packet->data[8], sizeof(float));
		memcpy(&external_air_data_packet->airspeed, &an_packet->data[12], sizeof(float));
		memcpy(&external_air_data_packet->barometric_altitude_standard_deviation, &an_packet->data[16], sizeof(float));
		memcpy(&external_air_data_packet->airspeed_standard_deviation, &an_packet->data[20], sizeof(float));
		external_air_data_packet->flags.r = an_packet->data[24];
		return 0;
	}
	else return 1;
}

void encode_external_air_data_packet(an_packet_t* an_packet, external_air_data_packet_t* external_air_data_packet)
{
    an_packet->id = packet_id_external_air_data;
	an_packet->length = 25;
	memcpy(&an_packet->data[0], &external_air_data_packet->barometric_altitude_delay, sizeof(float));
	memcpy(&an_packet->data[4], &external_air_data_packet->airspeed_delay, sizeof(float));
	memcpy(&an_packet->data[8], &external_air_data_packet->barometric_altitude, sizeof(float));
	memcpy(&an_packet->data[12], &external_air_data_packet->airspeed, sizeof(float));
	memcpy(&an_packet->data[16], &external_air_data_packet->barometric_altitude_standard_deviation, sizeof(float));
	memcpy(&an_packet->data[20], &external_air_data_packet->airspeed_standard_deviation, sizeof(float));
	an_packet->data[24] = external_air_data_packet->flags.r;
}

int decode_gnss_information_packet(gnss_receiver_information_packet_t* gnss_information_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gnss_receiver_information && an_packet->length == 48)
	{
		gnss_information_packet->gnss_manufacturer_id = an_packet->data[0];
		gnss_information_packet->gnss_receiver_model = an_packet->data[1];
		memcpy(gnss_information_packet->serial_number, &an_packet->data[2], 11 * sizeof(char));
		gnss_information_packet->serial_number[10] = '\0';
		memcpy(&gnss_information_packet->firmware_version, &an_packet->data[12], sizeof(uint32_t));
		memcpy(&gnss_information_packet->software_license, &an_packet->data[16], 3 * sizeof(uint32_t));
		memcpy(&gnss_information_packet->omnistar_serial_number, &an_packet->data[28], sizeof(uint32_t));
		memcpy(&gnss_information_packet->omnistar_subscription_start_unix_time, &an_packet->data[32], sizeof(uint32_t));
		memcpy(&gnss_information_packet->omnistar_subscription_expiry_unix_time, &an_packet->data[36], sizeof(uint32_t));
		gnss_information_packet->omnistar_engine_mode = an_packet->data[40];
		gnss_information_packet->rtk_accuracy = an_packet->data[41];
		return 0;
	}
	else return 1;
}

int decode_raw_dvl_data_packet(raw_dvl_data_packet_t* raw_dvl_data_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_raw_dvl_data && an_packet->length == 60)
	{
		memcpy(&raw_dvl_data_packet->unix_timestamp, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&raw_dvl_data_packet->microseconds, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&raw_dvl_data_packet->status.r, &an_packet->data[8], sizeof(uint32_t));
		memcpy(&raw_dvl_data_packet->bottom_velocity[0], &an_packet->data[12], 3 * sizeof(float));
		memcpy(&raw_dvl_data_packet->bottom_velocity_standard_deviation, &an_packet->data[24], sizeof(float));
		memcpy(&raw_dvl_data_packet->water_velocity[0], &an_packet->data[28], 3 * sizeof(float));
		memcpy(&raw_dvl_data_packet->water_velocity_standard_deviation, &an_packet->data[40], sizeof(float));
		memcpy(&raw_dvl_data_packet->water_velocity_layer_depth, &an_packet->data[44], sizeof(float));
		memcpy(&raw_dvl_data_packet->depth, &an_packet->data[48], sizeof(float));
		memcpy(&raw_dvl_data_packet->altitude, &an_packet->data[52], sizeof(float));
		memcpy(&raw_dvl_data_packet->temperature, &an_packet->data[56], sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_north_seeking_status_packet(north_seeking_status_packet_t* north_seeking_status_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_north_seeking_status && an_packet->length == 28)
	{
		memcpy(&north_seeking_status_packet->north_seeking_status.r, &an_packet->data[0], sizeof(uint16_t));
		memcpy(&north_seeking_status_packet->version, &an_packet->data[2], sizeof(uint16_t));
		north_seeking_status_packet->progress = an_packet->data[4];
		north_seeking_status_packet->alignment_attempts = an_packet->data[5];
		memcpy(&north_seeking_status_packet->coarse_alignment_heading, &an_packet->data[8], sizeof(float));
		memcpy(&north_seeking_status_packet->predicted_accuracy, &an_packet->data[12], sizeof(float));
		return 0;
	}
	else return 1;
}


int decode_gimbal_state_packet(gimbal_state_packet_t* gimbal_state_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gimbal_state && an_packet->length == 8)
	{
		memcpy(&gimbal_state_packet->current_angle, &an_packet->data[0], sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_gimbal_state_packet(an_packet_t* an_packet, gimbal_state_packet_t* gimbal_state_packet)
{
    an_packet->id = packet_id_gimbal_state;
	an_packet->length = 8;
	memcpy(&an_packet->data[0], &gimbal_state_packet->current_angle, sizeof(float));
	memset(&an_packet->data[4], 0, 4 * sizeof(uint8_t));
}

int decode_automotive_packet(automotive_packet_t* automotive_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_automotive && an_packet->length == 24)
	{
		memcpy(&automotive_packet->virtual_odometer_distance, &an_packet->data[0], sizeof(float));
		memcpy(&automotive_packet->slip_angle, &an_packet->data[4], sizeof(float));
		memcpy(&automotive_packet->velocity_x, &an_packet->data[8], sizeof(float));
		memcpy(&automotive_packet->velocity_y, &an_packet->data[12], sizeof(float));
		memcpy(&automotive_packet->distance_standard_deviation, &an_packet->data[16], sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_external_magnetometers_packet(external_magnetometers_packet_t* external_magnetometers_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_external_magnetometers && an_packet->length == 17)
	{
		memcpy(&external_magnetometers_packet->delay, &an_packet->data[0], sizeof(float));
		memcpy(&external_magnetometers_packet->magnetometer[0], &an_packet->data[4], 3 * sizeof(float));
		memcpy(&external_magnetometers_packet->flags, &an_packet->data[16], sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

void encode_external_magnetometers_packet(an_packet_t* an_packet, external_magnetometers_packet_t* external_magnetometers_packet)
{
    an_packet->id = packet_id_external_magnetometers;
	an_packet->length = 17;
	memcpy(&an_packet->data[0], &external_magnetometers_packet->delay, sizeof(float));
	memcpy(&an_packet->data[4], &external_magnetometers_packet->magnetometer[0], 3 * sizeof(float));
	memcpy(&an_packet->data[16], &external_magnetometers_packet->flags, sizeof(uint8_t));
}

void encode_zero_angular_velocity_packet(an_packet_t* an_packet, zero_angular_velocity_packet_t* zero_angular_velocity_packet)
{
    an_packet->id = packet_id_zero_angular_velocity;
	an_packet->length = 8;
	memcpy(&an_packet->data[0], &zero_angular_velocity_packet->duration, sizeof(float));
}

int decode_extended_satellites_packet(extended_satellites_packet_t* extended_satellites_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_extended_satellites && (an_packet->length - 2) % 9 == 0)
	{
		extended_satellites_packet->total_number_of_packets = an_packet->data[0];
		extended_satellites_packet->packet_number = an_packet->data[1];
		int number_of_satellites = (an_packet->length - 2) / 9;
		for(int i = 0; i < MAXIMUM_EXTENDED_SATELLITES; i++)
		{
			if(i < number_of_satellites)
			{
				extended_satellites_packet->extended_satellites[i].satellite_system = an_packet->data[9 * i + 2];
				extended_satellites_packet->extended_satellites[i].number = an_packet->data[9 * i + 3];
				extended_satellites_packet->extended_satellites[i].frequencies.r = an_packet->data[9 * i + 4];
				extended_satellites_packet->extended_satellites[i].elevation = an_packet->data[9 * i + 5];
				memcpy(&extended_satellites_packet->extended_satellites[i].azimuth, &an_packet->data[9 * i + 6], sizeof(uint16_t));
				extended_satellites_packet->extended_satellites[i].snr1 = an_packet->data[9 * i + 8];
				extended_satellites_packet->extended_satellites[i].snr2 = an_packet->data[9 * i + 9];
				memcpy(&extended_satellites_packet->extended_satellites[i].flags, &an_packet->data[9 * i + 10], sizeof(uint8_t));
			}
			else memset(&extended_satellites_packet->extended_satellites[i], 0, sizeof(extended_satellite_t));
		}
		return 0;
	}
	else return 1;
}


// ------------------- Sensor temperatures ------------- //
int decode_sensor_temperatures_packet(sensor_temperature_packet_t* sensor_temperature_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_sensor_temperatures && an_packet->length == 32)
	{
		memcpy(&sensor_temperature_packet->accelerometer_temperature[0], &an_packet->data[0], 3 * sizeof(float));
		memcpy(&sensor_temperature_packet->gyroscope_temperature[0], &an_packet->data[12], 3 * sizeof(float));
		memcpy(&sensor_temperature_packet->magnetometer_temperature, &an_packet->data[24], sizeof(float));
		memcpy(&sensor_temperature_packet->pressure_sensor_temperature, &an_packet->data[28], sizeof(float));
		return 0;
	}
	else return 1;
}

static gps_mask_t anpp_sensor_temperatures(struct gps_device_t *session, an_packet_t* an_packet) {
  gps_mask_t mask = 0;

  sensor_temperatures_packet_t sensor_temperatures_packet;
  
  if (decode_sensor_temperatures_packet(&sensor_temperatures_packet, an_packet) == 0) {
    for (int i=0; i<3; i++) {
      session->driver.anpp.gyroscope_temperature[i] = sensor_temperature_packet->gyroscope_temperature[i];
      session->driver.anpp.accelerometer_temperature[i] = sensor_temperature_packet[i]->accelerometer_temperature[i];   
    }
    session->driver.anpp.pressure_temperature = sensor_temperature_packet->pressure_sensor_temperature;

    GPSD_LOG(LOG_PROG, &session->context->errout,
	     "ANPP: Sensor temperatures (deg C): gyro %.1f %.1f %.1f"
	     " accel %.1f %.1f %.1f pressure %.1f\n",
	     session->driver.anpp.gyroscope_temperature[0],
	     session->driver.anpp.gyroscope_temperature[1],
	     session->driver.anpp.gyroscope_temperature[2],
	     session->driver.anpp.accelerometer_temperature[0],
	     session->driver.anpp.accelerometer_temperature[1],
	     session->driver.anpp.accelerometer_temperature[2],
	     session->driver.anpp.pressure_temperature);
  }
  else {
    GPSD_LOG(LOG_WARN, &session->context->errout,
	     "ANPP: Sensor temperatures: unable to decode");
    return 0;
  }
  
  return mask;  
}
// -------------------------------------------------------- //


// ------------------------ System temperature -------------//
int decode_system_temperature_packet(system_temperature_packet_t* system_temperature_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_system_temperature && an_packet->length == 64)
	{
		memcpy(&system_temperature_packet->system_temperature, &an_packet->data[0], sizeof(float));
		return 0;
	}
	else return 1;
}

static gps_mask_t anpp_system_temperature(struct gps_device_t *session, an_packet_t* an_packet) {
  gps_mask_t mask = 0;

  system_temperature_packet_t system_temperature_packet;
  
  if (decode_system_temperature_packet(&system_temperature_packet, an_packet) == 0) {
   
    session->driver.anpp.system_temperature = system_temperature_packet->system_temperature;

    GPSD_LOG(LOG_PROG, &session->context->errout,
	     "ANPP: System temperature (deg C): %.1f\n",
	     session->driver.anpp.system_temperature);
  }
  else {
    GPSD_LOG(LOG_WARN, &session->context->errout,
	     "ANPP: System temperature: unable to decode");
    return 0;
  }
  
  return mask;  
}
// --------------------------------------------------------- //

int decode_vessel_motion_packet(vessel_motion_packet_t* vessel_motion_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_vessel_motion && an_packet->length == 48)
	{
		memcpy(&vessel_motion_packet->surge_points, &an_packet->data[0], 4 * sizeof(float));
		memcpy(&vessel_motion_packet->sway_points, &an_packet->data[16], 4 * sizeof(float));
		memcpy(&vessel_motion_packet->heave_points, &an_packet->data[32], 4 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_packet_timer_period_packet(packet_timer_period_packet_t* packet_timer_period_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_packet_timer_period && an_packet->length == 4)
	{
		packet_timer_period_packet->permanent = an_packet->data[0];
		packet_timer_period_packet->utc_synchronisation = an_packet->data[1];
		memcpy(&packet_timer_period_packet->packet_timer_period, &an_packet->data[2], sizeof(uint16_t));
		return 0;
	}
	else return 1;
}

void encode_packet_timer_period_packet(an_packet_t* an_packet, packet_timer_period_packet_t* packet_timer_period_packet)
{
    an_packet->id = packet_id_packet_timer_period;
	an_packet->length = 4;
	an_packet->data[0] = packet_timer_period_packet->permanent > 0;
	an_packet->data[1] = packet_timer_period_packet->utc_synchronisation > 0;
	memcpy(&an_packet->data[2], &packet_timer_period_packet->packet_timer_period, sizeof(uint16_t));
}

int decode_packet_periods_packet(packet_periods_packet_t* packet_periods_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_packet_periods && (an_packet->length - 2) % 5 == 0)
	{
		int packet_periods_count = (an_packet->length - 2) / 5;
		packet_periods_packet->permanent = an_packet->data[0];
		packet_periods_packet->clear_existing_packets = an_packet->data[1];
		for(int i = 0; i < MAXIMUM_PACKET_PERIODS; i++)
		{
			if(i < packet_periods_count)
			{
				packet_periods_packet->packet_periods[i].packet_id = an_packet->data[2 + 5 * i];
				memcpy(&packet_periods_packet->packet_periods[i].period, &an_packet->data[2 + 5 * i + 1], sizeof(uint32_t));
			}
			else memset(&packet_periods_packet->packet_periods[i], 0, sizeof(packet_period_t));
		}
		return 0;
	}
	else return 1;
}

void encode_packet_periods_packet(an_packet_t* an_packet, packet_periods_packet_t* packet_periods_packet)
{
	int i;
    an_packet->id = packet_id_packet_periods;
	an_packet->length = 252;
	an_packet->data[0] = packet_periods_packet->permanent > 0;
	an_packet->data[1] = packet_periods_packet->clear_existing_packets;
	for(i = 0; i < MAXIMUM_PACKET_PERIODS; i++)
	{
		if(packet_periods_packet->packet_periods[i].packet_id)
		{
			an_packet->data[2 + 5 * i] = packet_periods_packet->packet_periods[i].packet_id;
			memcpy(&an_packet->data[2 + 5 * i + 1], &packet_periods_packet->packet_periods[i].period, sizeof(uint32_t));
		}
		else break;
	}
	an_packet->length = 2 + 5 * i;
}

int decode_baud_rates_packet(baud_rates_packet_t* baud_rates_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_baud_rates && an_packet->length == 17)
	{
		baud_rates_packet->permanent = an_packet->data[0];
		memcpy(&baud_rates_packet->primary_baud_rate, &an_packet->data[1], sizeof(uint32_t));
		memcpy(&baud_rates_packet->gpio_1_2_baud_rate, &an_packet->data[5], sizeof(uint32_t));
		memcpy(&baud_rates_packet->auxiliary_baud_rate, &an_packet->data[9], sizeof(uint32_t));
		memcpy(&baud_rates_packet->reserved, &an_packet->data[13], sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

void encode_baud_rates_packet(an_packet_t* an_packet, baud_rates_packet_t* baud_rates_packet)
{
    an_packet->id = packet_id_baud_rates;
	an_packet->length = 17;
	an_packet->data[0] = baud_rates_packet->permanent;
	memcpy(&an_packet->data[1], &baud_rates_packet->primary_baud_rate, sizeof(uint32_t));
	memcpy(&an_packet->data[5], &baud_rates_packet->gpio_1_2_baud_rate, sizeof(uint32_t));
	memcpy(&an_packet->data[9], &baud_rates_packet->auxiliary_baud_rate, sizeof(uint32_t));
	memcpy(&an_packet->data[13], &baud_rates_packet->reserved, sizeof(uint32_t));
}

int decode_installation_alignment_packet(installation_alignment_packet_t* installation_alignment_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_installation_alignment && an_packet->length == 73)
	{
		installation_alignment_packet->permanent = an_packet->data[0];
		memcpy(&installation_alignment_packet->alignment_dcm[0][0], &an_packet->data[1], 9 * sizeof(float));
		memcpy(&installation_alignment_packet->gnss_antenna_offset[0], &an_packet->data[37], 3 * sizeof(float));
		memcpy(&installation_alignment_packet->odometer_offset[0], &an_packet->data[49], 3 * sizeof(float));
		memcpy(&installation_alignment_packet->external_data_offset[0], &an_packet->data[61], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_installation_alignment_packet(an_packet_t* an_packet, installation_alignment_packet_t* installation_alignment_packet)
{
    an_packet->id = packet_id_installation_alignment;
	an_packet->length = 73;
	an_packet->data[0] = installation_alignment_packet->permanent;
	memcpy(&an_packet->data[1], &installation_alignment_packet->alignment_dcm[0][0], 9 * sizeof(float));
	memcpy(&an_packet->data[37], &installation_alignment_packet->gnss_antenna_offset[0], 3 * sizeof(float));
	memcpy(&an_packet->data[49], &installation_alignment_packet->odometer_offset[0], 3 * sizeof(float));
	memcpy(&an_packet->data[61], &installation_alignment_packet->external_data_offset[0], 3 * sizeof(float));
}

int decode_filter_options_packet(filter_options_packet_t* filter_options_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_filter_options && an_packet->length == 17)
	{
		memcpy(filter_options_packet, &an_packet->data[0], 9 * sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

void encode_filter_options_packet(an_packet_t* an_packet, filter_options_packet_t* filter_options_packet)
{
    an_packet->id = packet_id_filter_options;
	an_packet->length = 17;
	memcpy(&an_packet->data[0], filter_options_packet, 9 * sizeof(uint8_t));
	memset(&an_packet->data[9], 0, 8 * sizeof(uint8_t));
}

int decode_gpio_configuration_packet(gpio_configuration_packet_t* gpio_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gpio_configuration && an_packet->length == 13)
	{
		gpio_configuration_packet->permanent = an_packet->data[0];
		memcpy(gpio_configuration_packet->gpio_function, &an_packet->data[1], 4 * sizeof(uint8_t));
		gpio_configuration_packet->gpio_voltage_selection = an_packet->data[5];
		return 0;
	}
	else return 1;
}

void encode_gpio_configuration_packet(an_packet_t* an_packet, gpio_configuration_packet_t* gpio_configuration_packet)
{
    an_packet->id = packet_id_gpio_configuration;
	an_packet->length = 13;
	an_packet->data[0] = gpio_configuration_packet->permanent;
	memcpy(&an_packet->data[1], gpio_configuration_packet->gpio_function, 4 * sizeof(uint8_t));
	an_packet->data[5] = gpio_configuration_packet->gpio_voltage_selection;
	memset(&an_packet->data[6], 0, 7 * sizeof(uint8_t));
}

int decode_odometer_configuration_packet(odometer_configuration_packet_t* odometer_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_odometer_configuration && an_packet->length == 8)
	{
		odometer_configuration_packet->permanent = an_packet->data[0];
		odometer_configuration_packet->automatic_calibration = an_packet->data[1];
		memcpy(&odometer_configuration_packet->pulse_length, &an_packet->data[4], sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_odometer_configuration_packet(an_packet_t* an_packet, odometer_configuration_packet_t* odometer_configuration_packet)
{
    an_packet->id = packet_id_odometer_configuration;
	an_packet->length = 8;
	an_packet->data[0] = odometer_configuration_packet->permanent;
	an_packet->data[1] = odometer_configuration_packet->automatic_calibration;
	memset(&an_packet->data[2], 0, 2 * sizeof(uint8_t));
	memcpy(&an_packet->data[4], &odometer_configuration_packet->pulse_length, sizeof(float));
}

void encode_zero_alignment_packet(an_packet_t* an_packet, zero_alignment_packet_t* zero_alignment_packet)
{
	uint32_t verification = 0x9A4E8055;
    an_packet->id = packet_id_zero_alignment;
	an_packet->length = 5;
	an_packet->data[0] = zero_alignment_packet->permanent;
	memcpy(&an_packet->data[1], &verification, sizeof(uint32_t));
}

int decode_heave_offset_packet(heave_offset_packet_t* heave_offset_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_reference_offsets && an_packet->length == 49)
	{
		heave_offset_packet->permanent = an_packet->data[0];
		memcpy(&heave_offset_packet->heave_point_1_offset[0], &an_packet->data[1], 3 * sizeof(float));
		memcpy(&heave_offset_packet->heave_point_2_offset[0], &an_packet->data[13], 3 * sizeof(float));
		memcpy(&heave_offset_packet->heave_point_3_offset[0], &an_packet->data[25], 3 * sizeof(float));
		memcpy(&heave_offset_packet->heave_point_4_offset[0], &an_packet->data[37], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_heave_offset_packet(an_packet_t* an_packet, heave_offset_packet_t* heave_offset_packet)
{
    an_packet->id = packet_id_reference_offsets;
	an_packet->length = 49;
	an_packet->data[0] = heave_offset_packet->permanent;
	memcpy(&an_packet->data[1], &heave_offset_packet->heave_point_1_offset[0], 3 * sizeof(float));
	memcpy(&an_packet->data[13], &heave_offset_packet->heave_point_2_offset[0], 3 * sizeof(float));
	memcpy(&an_packet->data[25], &heave_offset_packet->heave_point_3_offset[0], 3 * sizeof(float));
	memcpy(&an_packet->data[37], &heave_offset_packet->heave_point_4_offset[0], 3 * sizeof(float));
}

int decode_gpio_output_configuration_packet(gpio_output_configuration_packet_t* gpio_output_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gpio_output_configuration && an_packet->length == 183)
	{
		gpio_output_configuration_packet->permanent = an_packet->data[0];
		memcpy(&gpio_output_configuration_packet->auxiliary_port, &an_packet->data[1], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->gpio_port, &an_packet->data[27], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->logging_port, &an_packet->data[53], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->data_port_1, &an_packet->data[79], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->data_port_2, &an_packet->data[105], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->data_port_3, &an_packet->data[131], 18 * sizeof(uint8_t));
		memcpy(&gpio_output_configuration_packet->data_port_4, &an_packet->data[157], 18 * sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

void encode_gpio_output_configuration_packet(an_packet_t* an_packet, gpio_output_configuration_packet_t* gpio_output_configuration_packet)
{
	an_packet->id = packet_id_gpio_output_configuration;
	an_packet->length = 183;
	if(an_packet != NULL)
	{
		an_packet->data[0] = gpio_output_configuration_packet->permanent;
		memcpy(&an_packet->data[1], &gpio_output_configuration_packet->auxiliary_port, 18 * sizeof(uint8_t));
		memset(&an_packet->data[19], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[27], &gpio_output_configuration_packet->gpio_port, 18 * sizeof(uint8_t));
		memset(&an_packet->data[45], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[53], &gpio_output_configuration_packet->logging_port, 18 * sizeof(uint8_t));
		memset(&an_packet->data[71], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[79], &gpio_output_configuration_packet->data_port_1, 18 * sizeof(uint8_t));
		memset(&an_packet->data[97], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[105], &gpio_output_configuration_packet->data_port_2, 18 * sizeof(uint8_t));
		memset(&an_packet->data[123], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[131], &gpio_output_configuration_packet->data_port_3, 18 * sizeof(uint8_t));
		memset(&an_packet->data[149], 0, 8 * sizeof(uint8_t));
		memcpy(&an_packet->data[157], &gpio_output_configuration_packet->data_port_4, 18 * sizeof(uint8_t));
		memset(&an_packet->data[175], 0, 8 * sizeof(uint8_t));
	}
}

int decode_dual_antenna_configuration_packet(dual_antenna_configuration_packet_t* dual_antenna_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_dual_antenna_configuration && an_packet->length == 17)
	{
		dual_antenna_configuration_packet->permanent = an_packet->data[0];
		memcpy(&dual_antenna_configuration_packet->options.r, &an_packet->data[1], sizeof(uint16_t));
		dual_antenna_configuration_packet->automatic_offset_orientation = an_packet->data[3];
		dual_antenna_configuration_packet->reserved = an_packet->data[4];
		memcpy(&dual_antenna_configuration_packet->manual_offset, &an_packet->data[5], 3 * sizeof(float));
		return 0;
	}
	return 1;
}

void encode_dual_antenna_configuration_packet(an_packet_t* an_packet, dual_antenna_configuration_packet_t* dual_antenna_configuration_packet)
{
    an_packet->id = packet_id_dual_antenna_configuration;
	an_packet->length = 17;
	an_packet->data[0] = dual_antenna_configuration_packet->permanent;
	memcpy(&an_packet->data[1], &dual_antenna_configuration_packet->options.r, sizeof(uint16_t));
	an_packet->data[3] = dual_antenna_configuration_packet->automatic_offset_orientation;
	an_packet->data[4] = 0;
	memcpy(&an_packet->data[5], &dual_antenna_configuration_packet->manual_offset, 3 * sizeof(float));
}

int decode_gnss_configuration_packet(gnss_configuration_packet_t* gnss_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gnss_configuration && an_packet->length == 85)
	{
		gnss_configuration_packet->permanent = an_packet->data[0];
		memcpy(&gnss_configuration_packet->gnss_frequencies, &an_packet->data[1], sizeof(uint64_t));
		memcpy(&gnss_configuration_packet->pdop, &an_packet->data[9], sizeof(float));
		memcpy(&gnss_configuration_packet->tdop, &an_packet->data[13], sizeof(float));
		gnss_configuration_packet->elevation_mask = an_packet->data[17];
		gnss_configuration_packet->snr_mask = an_packet->data[18];
		gnss_configuration_packet->sbas_corrections_enabled = an_packet->data[19];
		gnss_configuration_packet->lband_mode = an_packet->data[20];
		memcpy(&gnss_configuration_packet->lband_frequency, &an_packet->data[21], sizeof(uint32_t));
		memcpy(&gnss_configuration_packet->lband_baud, &an_packet->data[25], sizeof(uint32_t));
		memcpy(&gnss_configuration_packet->primary_antenna_type, &an_packet->data[29], sizeof(uint32_t));
		memcpy(&gnss_configuration_packet->secondary_antenna_type, &an_packet->data[33], sizeof(uint32_t));
		gnss_configuration_packet->lband_satellite_id = an_packet->data[37];
		return 0;
	}
	return 1;
}

void encode_gnss_configuration_packet(an_packet_t* an_packet, gnss_configuration_packet_t* gnss_configuration_packet)
{
    an_packet->id = packet_id_gnss_configuration;
	an_packet->length = 85;
	an_packet->data[0] = gnss_configuration_packet->permanent;
	memcpy(&an_packet->data[1], &gnss_configuration_packet->gnss_frequencies, sizeof(uint64_t));
	memcpy(&an_packet->data[9], &gnss_configuration_packet->pdop, sizeof(float));
	memcpy(&an_packet->data[13], &gnss_configuration_packet->tdop, sizeof(float));
	an_packet->data[17] = gnss_configuration_packet->elevation_mask;
	an_packet->data[18] = gnss_configuration_packet->snr_mask;
	an_packet->data[19] = gnss_configuration_packet->sbas_corrections_enabled;
	an_packet->data[20] = gnss_configuration_packet->lband_mode;
	memcpy(&an_packet->data[21], &gnss_configuration_packet->lband_frequency, sizeof(uint32_t));
	memcpy(&an_packet->data[25], &gnss_configuration_packet->lband_baud, sizeof(uint32_t));
	memcpy(&an_packet->data[29], &gnss_configuration_packet->primary_antenna_type, sizeof(uint32_t));
	memcpy(&an_packet->data[33], &gnss_configuration_packet->secondary_antenna_type, sizeof(uint32_t));
	an_packet->data[37] = gnss_configuration_packet->lband_satellite_id;
}

int decode_user_data_packet(user_data_packet_t* user_data_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_user_data && an_packet->length == 64)
	{
		memcpy(&user_data_packet->user_data, &an_packet->data[0], 64);
		return 0;
	}
	return 1;
}

void encode_user_data_packet(an_packet_t* an_packet, user_data_packet_t* user_data_packet)
{
    an_packet->id = packet_id_user_data;
	an_packet->length = 64;
	memcpy(&an_packet->data[0], &user_data_packet->user_data, 64);
}

int decode_gpio_input_configuration_packet(gpio_input_configuration_packet_t* gpio_input_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_gpio_input_configuration && an_packet->length == 65)
	{
		gpio_input_configuration_packet->permanent = an_packet->data[0];
		memcpy(&gpio_input_configuration_packet->gimbal_radians_per_encoder_tick, &an_packet->data[1], sizeof(float));
		return 0;
	}
	return 1;
}

void encode_gpio_input_configuration_packet(an_packet_t* an_packet, gpio_input_configuration_packet_t* gpio_input_configuration_packet)
{
    an_packet->id = packet_id_gpio_input_configuration;
	an_packet->length = 65;
	an_packet->data[0] = gpio_input_configuration_packet->permanent;
	memcpy(&an_packet->data[1], &gpio_input_configuration_packet->gimbal_radians_per_encoder_tick, sizeof(float));
}

int decode_ip_dataports_configuration_packet(ip_dataports_configuration_packet_t* ip_dataports_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_ip_dataports_configuration && an_packet->length == 30)
	{
		for(int i = 0; i < 4; i++)
		{
			memcpy(&ip_dataports_configuration_packet->ip_dataport_configuration[i].ip_address, &an_packet->data[7 * i + 2], sizeof(uint32_t));
			memcpy(&ip_dataports_configuration_packet->ip_dataport_configuration[i].port, &an_packet->data[7 * i + 6], sizeof(uint16_t));
			ip_dataports_configuration_packet->ip_dataport_configuration[i].ip_dataport_mode = an_packet->data[7 * i + 8];
		}
		return 0;
	}
	return 1;
}

void encode_ip_dataports_configuration_packet(an_packet_t* an_packet, ip_dataports_configuration_packet_t* ip_dataports_configuration_packet)
{
    an_packet->id = packet_id_ip_dataports_configuration;
	an_packet->length = 30;
	for(int i = 0; i < 4; i++)
	{
		memcpy(&an_packet->data[7 * i + 2], &ip_dataports_configuration_packet->ip_dataport_configuration[i].ip_address, sizeof(uint32_t));
		memcpy(&an_packet->data[7 * i + 6], &ip_dataports_configuration_packet->ip_dataport_configuration[i].port, sizeof(uint16_t));
		an_packet->data[7 * i + 8] = ip_dataports_configuration_packet->ip_dataport_configuration[i].ip_dataport_mode;
	}
}

int decode_can_configuration_packet(can_configuration_packet_t* can_configuration_packet, an_packet_t* an_packet)
{
	if(an_packet->id == packet_id_can_configuration && an_packet->length == 11)
	{
		can_configuration_packet->permanent = an_packet->data[0];
		can_configuration_packet->enabled = an_packet->data[1];
		memcpy(&can_configuration_packet->baud_rate, &an_packet->data[2], sizeof(uint32_t));
		can_configuration_packet->can_protocol = an_packet->data[6];
		return 0;
	}
	return 1;
}

void encode_can_configuration_packet(an_packet_t* an_packet, can_configuration_packet_t* can_configuration_packet)
{
    an_packet->id = packet_id_can_configuration;
	an_packet->length = 11;
	an_packet->data[0] = can_configuration_packet->permanent;
	an_packet->data[1] = can_configuration_packet->enabled;
	memcpy(&an_packet->data[2], &can_configuration_packet->baud_rate, sizeof(uint32_t));
	an_packet->data[6] = can_configuration_packet->can_protocol;
}


/*
 * Decode the navigation solution message
 */
static gps_mask_t anpp_msg_navsol(struct gps_device_t *session,
                                     unsigned char *buf, size_t data_len)
{
    gps_mask_t mask;
    int flags;
    double Px, Py, Pz, Vx, Vy, Vz;

    if (data_len != ANPP_NAVSOL_MSG_LEN)
        return 0;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "anpp NAVSOL - navigation data\n");
    /* if this protocol has a way to test message validity, use it */
    flags = GET_FLAGS();
    if ((flags & ANPP_SOLUTION_VALID) == 0)
        return 0;

    mask = ONLINE_SET;

    /* extract ECEF navigation solution here */
    /* or extract the local tangential plane (ENU) solution */
    [session->newdata.ecef.x,
    session->newdata.ecef.y,
    session->newdata.ecef.z,
    session->newdata.ecef.vx,
    session->newdata.ecef.vy,
    session->newdata.ecef.vz] = GET_ECEF_FIX();
    mask |= ECEF_SET | VECEF_SET;

    session->newdata.epx = GET_LONGITUDE_ERROR();
    session->newdata.epy = GET_LATITUDE_ERROR();
    session->newdata.eps = GET_SPEED_ERROR();
    session->gpsdata.satellites_used = GET_SATELLITES_USED();
    /*
     * Do *not* clear DOPs in a navigation solution message;
     * instead, opportunistically pick up whatever it gives
     * us and replace whatever values we computed from the
     * visibility matrix for he last skyview. The reason to trust
     * the chip returns over what we compute is that some
     * chips have internal deweighting albums to throw out sats
     * that increase DOP.
     */
    session->gpsdata.dop.hdop = GET_HDOP();
    session->gpsdata.dop.vdop = GET_VDOP();
    /* other DOP if available */
    mask |= DOP_SET;

    session->newdata.mode = GET_FIX_MODE();
    session->gpsdata.status = GET_FIX_STATUS();

    /*
     * Mix in CLEAR_IS to clue the daemon in about when to clear fix
     * information.  Mix in REPORT_IS when the sentence is reliably
     * the last in a reporting cycle.
     */
    mask |= MODE_SET | STATUS_SET | REPORT_IS;

    /*
     * At the end of each packet-cracking function, report at LOG_DATA level
     * the fields it potentially set and the transfer mask. Doing this
     * makes it relatively easy to track down data-management problems.
     */
    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NAVSOL: time=%.2f, ecef x:%.2f y: %.2f z: %.2f mode=%d "
             "status=%d\n",
             session->newdata.time,
             session->newdata.ecef.x,
             session->newdata.ecef.y,
             session->newdata.ecef.z,
             session->newdata.longitude,
             session->newdata.mode,
             session->gpsdata.status);

    return mask;
}

/**
 * GPS Leap Seconds
 */
static gps_mask_t anpp_msg_utctime(struct gps_device_t *session,
                                      unsigned char *buf, size_t data_len)
{
    double t;

    if (data_len != UTCTIME_MSG_LEN)
        return 0;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "anpp UTCTIME - navigation data\n");
    /* if this protocol has a way to test message validity, use it */
    flags = GET_FLAGS();
    if ((flags & ANPP_TIME_VALID) == 0)
        return 0;

    tow = GET_MS_TIMEOFWEEK();
    gps_week = GET_WEEKNUMBER();
    session->context->leap_seconds = GET_GPS_LEAPSECONDS();
    session->newdata.time = gpsd_gpstime_resolv(session, gps_week, tow);

    return TIME_SET | NTPTIME_IS | ONLINE_SET;
}

/**
 * GPS Satellite Info
 */
static gps_mask_t anpp_msg_svinfo(struct gps_device_t *session,
                                     unsigned char *buf, size_t data_len)
{
    unsigned char i, st, nchan, nsv;
    unsigned int tow;

    if (data_len != SVINFO_MSG_LEN )
        return 0;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "anpp SVINFO - navigation data\n");
    /* if this protocol has a way to test message validity, use it */
    flags = GET_FLAGS();
    if ((flags & ANPP_SVINFO_VALID) == 0)
        return 0;

    /*
     * some protocols have a variable length message listing only visible
     * satellites, even if there are less than the number of channels. others
     * have a fixed length message and send empty records for idle channels
     * that are not tracking or searching. whatever the case, nchan should
     * be set to the number of satellites which might be visible.
     */
    nchan = GET_NUMBER_OF_CHANNELS();
    if ((nchan < 1) || (nchan > MAXCHANNELS)) {
        GPSD_LOG(LOG_INF, &session->context->errout,
                 "too many channels reported\n");
        return 0;
    }
    gpsd_zero_satellites(&session->gpsdata);
    nsv = 0; /* number of actually used satellites */
    for (i = st = 0; i < nchan; i++) {
        /* get info for one channel/satellite */
        int off = GET_CHANNEL_STATUS(i);

        session->gpsdata.PRN[i]         = PRN_THIS_CHANNEL_IS_TRACKING(i);
        session->gpsdata.ss[i]          = (float)SIGNAL_STRENGTH_FOR_CHANNEL(i);
        session->gpsdata.elevation[i]   = SV_ELEVATION_FOR_CHANNEL(i);
        session->gpsdata.azimuth[i]     = SV_AZIMUTH_FOR_CHANNEL(i);

        if (CHANNEL_USED_IN_SOLUTION(i))
            session->gpsdata.used[nsv++] = session->gpsdata.PRN[i];

        if(session->gpsdata.PRN[i])
                st++;
    }
    /* if the satellite-info setence gives you UTC time, use it */
    session->gpsdata.skyview_time = NaN;
    session->gpsdata.satellites_used = nsv;
    session->gpsdata.satellites_visible = st;
    GPSD_LOG(LOG_DATA, &session->context->errout,
             "SVINFO: visible=%d used=%d mask={SATELLITE|USED}\n",
             session->gpsdata.satellites_visible,
             session->gpsdata.satellites_used);
    return SATELLITE_SET | USED_IS;
}

/**
 * Raw measurements
 */
static gps_mask_t anpp_msg_raw(struct gps_device_t *session,
                                  unsigned char *buf, size_t data_len)
{
    unsigned char i, st, nchan, nsv;
    unsigned int tow;

    if (data_len != RAW_MSG_LEN )
        return 0;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "anpp RAW - raw measurements\n");
    /* if this protocol has a way to test message validity, use it */
    flags = GET_FLAGS();
    if ((flags & ANPP_SVINFO_VALID) == 0)
        return 0;

    /*
     * not all chipsets emit the same information. some of these observables
     * can be easily converted into others. these are suggestions for the
     * quantities you may wish to try extract. chipset documentation may say
     * something like "this message contains information required to generate
     * a RINEX file." assign NAN for unavailable data.
     */
    nchan = GET_NUMBER_OF_CHANNELS();
    if ((nchan < 1) || (nchan > MAXCHANNELS)) {
        GPSD_LOG(LOG_INF, &session->context->errout,
                 "too many channels reported\n");
        return 0;
    }

    DTONS(&session->raw.mtime, GET_TIME());

    /* this is so we can tell which never got set */
    for (i = 0; i < MAXCHANNELS; i++)
        session->gpsdata.raw.meas[i].svid = 0;
    for (i = 0; i < n; i++){
        session->gpsdata.PRN[i] = GET_PRN();
        session->gpsdata.ss[i] = GET_SIGNAL()
        session->gpsdata.raw.meas[i].satstat = GET_FLAGS();
        session->gpsdata.raw.meas[i].pseudorange = GET_PSEUDORANGE();
        session->gpsdata.raw.meas[i].doppler = GET_DOPPLER();
        session->gpsdata.raw.meas[i].carrierphase = GET_CARRIER_PHASE();
        session->gpsdata.raw.meas[i].codephase = GET_CODE_PHASE();
        session->gpsdata.raw.meas[i].deltarange = GET_DELTA_RANGE();
    }
    return RAW_IS;
}

/**
 * Parse the data from the device
 */
gps_mask_t anpp_dispatch(struct gps_device_t *session,
                            unsigned char *buf, size_t len)
{
    size_t i;
    int type, used, visible, retmask = 0;

    if (len == 0)
        return 0;

    /*
     * Set this if the driver reliably signals end of cycle.
     * The core library zeroes it just before it calls each driver's
     * packet analyzer.
     */
    session->cycle_end_reliable = true;
    if (msgid == MY_START_OF_CYCLE)
        retmask |= CLEAR_IS;
    else if (msgid == MY_END_OF_CYCLE)
        retmask |= REPORT_IS;

    /* Put the buffer to decode into an_decoder */
    an_decoder_t an_decoder;
    an_decoder_initialise(&an_decoder);
    an_decoder->buffer = &buf;
    
    an_packet_t* an_packet;
    
    /* increment the decode buffer length by the number of bytes received */
    an_decoder_increment(&an_decoder, len);
    
    // Decode the packet, and check which type of packet it is. 
    if ((an_packet = an_packet_decode(&an_decoder)) != NULL) {
      // Most of these will not be implemented, since we won't care about them. 
      switch (an_packet->id) {
      case packet_id_acknowledge:
	break;
      case packet_id_request:
	break;
      case packet_id_boot_mode:
	break;
      case packet_id_device_information:
	break;
      case packet_id_restore_factory_settings:
	break;
      case packet_id_reset:
	break;
      case packet_id_6_reserved:
	break;
      case packet_id_file_transfer_request:
	break;
      case packet_id_file_transfer_acknowledge:
	break;
      case packet_id_file_transfer:
	break;
      case packet_id_serial_port_passthrough:
	break;
      case packet_id_ip_configuration:
	break;
      case packet_id_12_reserved:
	break;
      case packet_id_extended_device_information:
	break;
      case packet_id_subcomponent_information:
	break;
      case end_system_packets:
	break;
      case packet_id_system_state:
	break;
      case packet_id_unix_time:
	mask = anpp_unix_time(session, an_packet);
	break;
      case packet_id_formatted_time:
	break;
      case packet_id_status:
	// mask = anpp_status(session, an_packet);
	break;
      case packet_id_position_standard_deviation:
	// mask = anpp_position_standard_deviation(session, an_packet);
	break;
      case packet_id_velocity_standard_deviation:
	break;
      case packet_id_euler_orientation_standard_deviation:
	mask = anpp_euler_orientation_standard_deviation(session, an_packet);
	break;
      case packet_id_quaternion_orientation_standard_deviation:
	break;
      case packet_id_raw_sensors:
	// mask = anpp_raw_sensors(session, an_packet);
	break;
      case packet_id_raw_gnss:
	// mask = anpp_raw_gnss(session, an_packet);
	break;
      case packet_id_satellites:
	// mask = anpp_satellites(session, an_packet);
	break;
      case packet_id_satellites_detailed:
	break;
      case packet_id_geodetic_position:
	// mask = anpp_geodetic_position(session, an_packet);
	break;
      case packet_id_ecef_position:
	break;
      case packet_id_utm_position:
	break;
      case packet_id_ned_velocity:
	// mask = anpp_ned_velocity(session, an_packet);
	break;
      case packet_id_body_velocity:
	// mask = anpp_body_velocity(session, an_packet);
	break;
      case packet_id_acceleration:
	// mask = anpp_acceleration(session, an_packet);
	break;
      case packet_id_body_acceleration:
	break;
      case packet_id_euler_orientation:
	mask = anpp_euler_orientation(session, an_packet);
	break;
      case packet_id_quaternion_orientation:
	break;
      case packet_id_dcm_orientation:
	break;
      case packet_id_angular_velocity:
	// mask = anpp_angular_velocity(session, an_packet);
	break;
      case packet_id_angular_acceleration:
	// mask = anpp_angular_acceleration(session, an_packet);
	break;
      case packet_id_external_position_velocity:
	break;
      case packet_id_external_position:
	break;
      case packet_id_external_velocity:
	break;
      case packet_id_external_body_velocity:
	break;
      case packet_id_external_heading:
	break;
      case packet_id_running_time:
	break;
      case packet_id_local_magnetics:
	break;
      case packet_id_odometer_state:
	break;
      case packet_id_external_time:
	break;
      case packet_id_external_depth:
	break;
      case packet_id_geoid_height:
	break;
      case packet_id_rtcm_corrections:
	break;
      case packet_id_56_reserved:
	break;
      case packet_id_wind:
	break;
      case packet_id_heave:
	break;
      case packet_id_59_reserved:
	break;
      case packet_id_raw_satellite_data:
	break;
      case packet_id_raw_satellite_ephemeris:
	break;
      case packet_id_62_reserved:
	break;
      case packet_id_63_reserved:
	break;
      case packet_id_64_reserved:
	break;
      case packet_id_65_reserved:
	break;
      case packet_id_66_reserved:
	break;
      case packet_id_external_odometer:
	break;
      case packet_id_external_air_data:
	break;
      case packet_id_gnss_receiver_information:
	break;
      case packet_id_raw_dvl_data:
	break;
      case packet_id_north_seeking_status:
	break;
      case packet_id_gimbal_state:
	break;
      case packet_id_automotive:
	break;
      case packet_id_74_reserved:
	break;
      case packet_id_external_magnetometers:
	break;
      case packet_id_76_reserved:
	break;
      case packet_id_77_reserved:
	break;
      case packet_id_78_reserved:
	break;
      case packet_id_79_reserved:
	break;
      case packet_id_basestation:
	break;
      case packet_id_81_reserved:
	break;
      case packet_id_82_reserved:
	break;
      case packet_id_zero_angular_velocity:
	break;
      case packet_id_extended_satellites:
	break;
      case packet_id_sensor_temperatures:
	mask = anpp_sensor_temperatures(session, an_packet);
	break;
      case packet_id_system_temperature:
	mask = anpp_system_temperatures(session, an_packet);
	break;
      case packet_id_quantum_sensor:
	break;
      case end_state_packets:
	break;
      case packet_id_packet_timer_period:
	break;
      case packet_id_packet_periods:
	break;
      case packet_id_baud_rates:
	break;
      case packet_id_183_reserved:
	break;
      case packet_id_sensor_ranges:
	break;
      case packet_id_installation_alignment:
	break;
      case packet_id_filter_options:
	break;
      case packet_id_187_reserved:
	break;
      case packet_id_gpio_configuration:
	break;
      case packet_id_magnetic_calibration_values:
	break;
      case packet_id_magnetic_calibration_configuration:
	break;
      case packet_id_magnetic_calibration_status:
	break;
      case packet_id_odometer_configuration:
	break;
      case packet_id_zero_alignment:
	break;
      case packet_id_reference_offsets:
	break;
      case packet_id_gpio_output_configuration:
	break;
      case packet_id_dual_antenna_configuration:
	break;
      case packet_id_gnss_configuration:
	break;
      case packet_id_user_data:
	break;
      case packet_id_gpio_input_configuration:
	break;
      case packet_id_200_reserved:
	break;
      case packet_id_201_reserved:
	break;
      case packet_id_ip_dataports_configuration:
	break;
      case packet_id_can_configuration:
	break;
      }
    }
    
    
    /* we may need to dump the raw packet */
    GPSD_LOG(LOG_RAW, &session->context->errout,
             "ANPP packet type 0x%02x\n", an_packet->id);

    switch (type)
    {
        /* Deliver message to specific decoder based on message type */

    default:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "unknown packet id %d length %d\n", type, len);
        return 0;
    }
}

/**********************************************************
 *
 * Externally called routines below here
 *
 **********************************************************/

static bool anpp_probe_detect(struct gps_device_t *session)
{
   /*
    * This method is used to elicit a positively identifying
    * response from a candidate device. Some drivers may use
    * this to test for the presence of a certain kernel module.
    */
   int test, satisfied;

   /* Your testing code here */
   test=satisfied=0;
   if (test==satisfied)
      return true;
   return false;
}

/**
 * Write data to the device, doing any required padding or checksumming
 */
static ssize_t anpp_control_send(struct gps_device_t *session,
                           char *msg, size_t msglen)
{
   bool ok;

   /* CONSTRUCT THE MESSAGE */

   /*
    * This copy to a public assembly buffer
    * enables gpsmon to snoop the control message
    * after it has been sent.
    */
   session->msgbuflen = msglen;
   (void)memcpy(session->msgbuf, msg, msglen);

   /* we may need to dump the message */
   GPSD_LOG(LOG_PROG, &session->context->errout,
               "writing anpp control type %02x\n");
   return gpsd_write(session, session->msgbuf, session->msgbuflen);
}

static void anpp_event_hook(struct gps_device_t *session, event_t event)
{
    if (session->context->readonly)
        return;

    if (event == EVENT_WAKEUP) {
       /*
        * Code to make the device ready to communicate.  Only needed if the
        * device is in some kind of sleeping state, and only shipped to
        * RS232C, so that gpsd won't send strings to unidentified USB devices
        * that might not be GPSes at all.
        */
    }
    if (event == EVENT_IDENTIFIED) {
        /*
         * Fires when the first full packet is recognized from a
         * previously unidentified device.  The session.lexer counter
         * is zeroed.  If your device has a default cycle time other
         * than 1 second, set session->device->gpsdata.cycle here. If
         * possible, get the software version and store it in
         * session->subtype.
         */
    }
    if (event == EVENT_CONFIGURE) {
        /*
         * Change sentence mix and set reporting modes as needed.
         * Called immediately after EVENT_IDENTIFIED fires, then just
         * after every packet received thereafter, but you probably
         * only want to take actions on the first few packets after
         * the session.lexer counter has been zeroed,
         *
         * Remember that session->lexer.counter is available when you
         * write this hook; you can use this fact to interleave configuration
         * sends with the first few packet reads, which is useful for
         * devices with small receive buffers.
         */
    } else if (event == EVENT_DRIVER_SWITCH) {
        /*
         * Fires when the driver on a device is changed *after* it
         * has been identified.
         */
    } else if (event == EVENT_DEACTIVATE) {
        /*
         * Fires when the device is deactivated.  Usr this to revert
         * whatever was done at EVENT_IDENTIFY and EVENT_CONFIGURE
         * time.
         */
    } else if (event == EVENT_REACTIVATE) {
       /*
        * Fires when a device is reactivated after having been closed.
        * Use this hook for re-establishing device settings that
        * it doesn't hold through closes.
        */
    }
}

/*
 * This is the entry point to the driver. When the packet sniffer recognizes
 * a packet for this driver, it calls this method which passes the packet to
 * the binary processor or the nmea processor, depending on the session type.
 */
static gps_mask_t anpp_parse_input(struct gps_device_t *session)
{
    if (ANPP_PACKET == session->lexer.type) {
        return anpp_dispatch(session, session->lexer.outbuffer,
                                session->lexer.outbuflen);
    }
    return generic_parse_input(session);
}

static bool anpp_set_speed(struct gps_device_t *session,
                              speed_t speed, char parity, int stopbits)
{
    /*
     * Set port operating mode, speed, parity, stopbits etc. here.
     * Note: parity is passed as 'N'/'E'/'O', but you should program
     * defensively and allow 0/1/2 as well.
     */
}

/*
 * Switch between NMEA and binary mode, if supported
 */
static void anpp_set_mode(struct gps_device_t *session, int mode)
{
    if (mode == MODE_NMEA) {
        /* send a mode switch control string */
    } else {
        /* send a mode switch control string */
    }
}

static double anpptime_offset(struct gps_device_t *session)
{
    /*
     * If NTP notification is enabled, the GPS will occasionally NTP
     * its notion of the time. This will lag behind actual time by
     * some amount which has to be determined by observation vs. (say
     * WWVB radio broadcasts) and, furthermore, may differ by baud
     * rate. This method is for computing the NTP fudge factor.  If
     * it's absent, an offset of 0.0 will be assumed, effectively
     * falling back on what's in ntp.conf. When it returns NAN,
     * nothing will be sent to NTP.
     */
    return MAGIC_CONSTANT;
}

static void anpp_wrapup(struct gps_device_t *session)
{
}

/* The methods in this code take parameters and have */
/* return values that conform to the requirements AT */
/* THE TIME THE CODE WAS WRITTEN.                    */
/*                                                   */
/* These values may well have changed by the time    */
/* you read this and methods could have been added   */
/* or deleted. Unused methods can be set to NULL.    */
/*                                                   */
/* The latest version can be found by inspecting   */
/* the contents of struct gps_type_t in gpsd.h.      */
/*                                                   */
/* This always contains the correct definitions that */
/* any driver must use to compile.                   */

/* This is everything we export */
/* *INDENT-OFF* */
const struct gps_type_t driver_anpp_binary = {
    /* Full name of type */
    .type_name        = "_proto",
    /* Associated lexer packet type */
    .packet_type      = ANPP_PACKET,
    /* Driver tyoe flags */
    .flags            = DRIVER_NOFLAGS,
    /* Response string that identifies device (not active) */
    .trigger          = NULL,
    /* Number of satellite channels supported by the device */
    .channels         = 12,
    /* Startup-time device detector */
    .probe_detect     = anpp_probe_detect,
    /* Packet getter (using default routine) */
    .get_packet       = packet_get1,
    /* Parse message packets */
    .parse_packet     = anpp_parse_input,
    /* RTCM handler (using default routine) */
    .rtcm_writer      = pass_rtcm,
    /* non-perturbing initial query (e.g. for version) */
    .init_query        = NULL,
    /* fire on various lifetime events */
    .event_hook       = anpp_event_hook,
    /* Speed (baudrate) switch */
    .speed_switcher   = anpp_set_speed,
    /* Switch to NMEA mode */
    .mode_switcher    = anpp_set_mode,
    /* Message delivery rate switcher (not active) */
    .rate_switcher    = NULL,
    /* Minimum cycle time of the device */
    .min_cycle        = 1,
    /* Control string sender - should provide checksum and headers/trailer */
    .control_send   = anpp_control_send,
    .time_offset     = anpptime_offset,
/* *INDENT-ON* */
};
#endif  // defined(ANPP_ENABLE)

// vim: set expandtab shiftwidth=4
