/*
  /*
  * Novatel format driver
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  /* must be before all includes */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#if defined(NOVATEL_ENABLE)

#include "../include/bits.h"

static  gps_mask_t novatel_parse_input(struct gps_device_t *);
static  gps_mask_t novatel_dispatch(struct gps_device_t *, unsigned char *,
                                    size_t );
static  gps_mask_t novatel_msg_navsol(struct gps_device_t *, unsigned char *,
                                      size_t );
static  gps_mask_t novatel_msg_utctime(struct gps_device_t *, unsigned char *,
                                       size_t );
static  gps_mask_t novatel_msg_svinfo(struct gps_device_t *, unsigned char *,
                                      size_t );
static  gps_mask_t novatel_msg_raw(struct gps_device_t *, unsigned char *,
                                   size_t );

/*
 * These methods may be called elsewhere in gpsd
 */
static  ssize_t novatel_control_send(struct gps_device_t *, char *, size_t);
static  bool novatel_probe_detect(struct gps_device_t *);
static  void novatel_event_hook(struct gps_device_t *, event_t);
static  bool novatel_set_speed(struct gps_device_t *, speed_t, char, int);
static  void novatel_set_mode(struct gps_device_t *, int);


/*
 * Novatel Packet Structure
 * ------------------------
 * Followed by packet data of the length specified in the header
 */


/*
 * From the Novatel OEM7 manual
 */
#define CRC32_POLYNOMIAL 0xEDB88320L
/* --------------------------------------------------------------------------
   Calculate a CRC value to be used by CRC calculation functions.
   -------------------------------------------------------------------------- */
unsigned long CRC32Value(int i) {
  int j;
  unsigned long ulCRC;
  ulCRC = i;
  for ( j = 8 ; j > 0; j-- ) {
    if ( ulCRC & 1 )
      ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
    else
      ulCRC >>= 1;
  }
  return ulCRC;
}
/* --------------------------------------------------------------------------
   Calculates the CRC-32 of a block of data all at once
   ulCount - Number of bytes in the data block
   ucBuffer - Data block
   -------------------------------------------------------------------------- */
unsigned long CalculateBlockCRC32( unsigned long ulCount, unsigned char
				   *ucBuffer ) {
  unsigned long ulTemp1;
  unsigned long ulTemp2;
  unsigned long ulCRC = 0;
  while ( ulCount-- != 0 ) {
    ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
    ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xFF );
    ulCRC = ulTemp1 ^ ulTemp2;
  }
  return( ulCRC );
}

/*
 * Novatel message -- INS Attitude message with short header
 * INSATTS
 */
static gps_mask_t insatts_message(struct gps_device_t *session, unsigned char *buf, size_t data_len) {
  gps_mask_t mask = 0;
  
  unsigned long week = getleu32(buf, NOVATEL_SHORT_HEADER_LENGTH);
  timespec_t seconds_into_week;
  DTOTS(&seconds_into_week, getled64(buf, NOVATEL_SHORT_HEADER_LENGTH+4));
  TS_NORM(&seconds_into_week);
  session->newdata.time = gpsd_gpstime_resolv(session, week, seconds_into_week);
  mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
  
  session->gpsdata.attitude.roll = getled64(buf, NOVATEL_SHORT_HEADER_LENGTH+12);
  session->gpsdata.attitude.pitch = getled64(buf, NOVATEL_SHORT_HEADER_LENGTH+20);
  session->gpsdata.attitude.heading = getled64(buf, NOVATEL_SHORT_HEADER_LENGTH+28);
  mask |= ATTITUDE_SET;

  //Status at H+36, 4 bytes

  GPSD_LOG(LOG_PROG, &session->context->errout,
	   "NOVATEL: Euler attitude: roll %.5f pitch %.5f heading %.5f\n",
	   session->gpsdata.attitude.roll,
	   session->gpsdata.attitude.pitch,
	   session->gpsdata.attitude.heading);
  
  return mask;  
}

/*
 * Novatel message -- INS Attitude and position standard deviation with short header
 * INSSTDEVS
 */
static gps_mask_t insstdevs_message(struct gps_device_t *session, unsigned char *buf, size_t data_len) {
  gps_mask_t mask = 0;
  
  unsigned long week = getleu32(buf, NOVATEL_SHORT_HEADER_LENGTH);
  timespec_t seconds_into_week;
  DTOTS(&seconds_into_week, getled64(buf, NOVATEL_SHORT_HEADER_LENGTH+4));
  TS_NORM(&seconds_into_week);
  session->newdata.time = gpsd_gpstime_resolv(session, week, seconds_into_week);
  mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;


  float latitude_std = getlef32(buf, NOVATEL_SHORT_HEADER_LENGTH);
  float longitude_std = getlef32(buf, NOVATEL_SHORT_HEADER_LENGTH+4);
  
  session->gpsdata.dop.xdop = latitude_std;
  session->gpsdata.dop.ydop = longitude_std;
  mask |= DOP_SET;
    
  session->newdata.eph = latitude_std > longitude_std ? latitude_std : longitude_std; 
  session->newdata.epv = getlef32(buf, NOVATEL_SHORT_HEADER_LENGTH+8); // Height above ellipsoid in meters
    
  mask |= HERR_SET;
  mask |= VERR_SET;

  GPSD_LOG(LOG_PROG, &session->context->errout,
	   "ANPP: Position standard deviation: eph %.3f epv %.3f\n",
	   session->newdata.eph,
	   session->newdata.epv);

  
  session->gpsdata.attitude.roll_std = getled64(buf, NOVATEL_SHORT_HEADER_LENGTH+24);
  session->gpsdata.attitude.pitch_std = getled64(buf, NOVATEL_SHORT_HEADER_LENGTH+28);
  session->gpsdata.attitude.heading_std = getled64(buf, NOVATEL_SHORT_HEADER_LENGTH+32);
  mask |= ATTITUDE_SET;

  //Status at H+36, 4 bytes

  GPSD_LOG(LOG_PROG, &session->context->errout,
	   "NOVATEL: Euler attitude: roll %.5f pitch %.5f heading %.5f\n",
	   session->gpsdata.attitude.roll,
	   session->gpsdata.attitude.pitch,
	   session->gpsdata.attitude.heading);
  
  return mask;  
}

/*
 * Novatel message -- position derived from the best source (INS, or GNSS)
 * BESTPOS
 */
static gps_mask_t bestpos_message(struct gps_device_t *session, unsigned char *buf, size_t data_len) {
  gps_mask_t mask = 0;
  // To be written
  return mask;
}


/*
 * Novatel message -- Heading derived from the vector between the two antennas
 * DUALANTENNAHEADING
 */
static gps_mask_t dualantennaheading_message(struct gps_device_t *session, unsigned char *buf, size_t data_len) {
  gps_mask_t mask = 0;
  // To be written
  return mask;
}

/*
 * Novatel message -- raw IMU data, corrected for gravity, earth rotation, sensor errors with short header
 * CORRIMUS
 */
static gps_mask_t corrimus_message(struct gps_device_t *session, unsigned char *buf, size_t data_len) {
  gps_mask_t mask = 0;
  // To be written
  return mask;
}

/*
 * Novatel message -- Temperatures, voltages, etc.
 * HWMONITOR
 */
static gps_mask_t hwmonitor_message(struct gps_device_t *session, unsigned char *buf, size_t data_len) {
  gps_mask_t mask = 0;
  // To be written
  return mask;
}

/**
 * Parse the data from the device
 */
gps_mask_t novatel_dispatch(struct gps_device_t *session,
                            unsigned char *buf, size_t len)
{
    size_t i;
    int used, visible, retmask = 0;

    uint8_t header_length = 0;
    uint8_t message_length = 0;
    uint16_t message_id = 0;
    if ( 0x12 == lexer->inbuffer[2] ) {
      // Long header
      message_id = lexer->inbuffer[3];
      message_id |= lexer->inbuffer[4] << 8;
      message_length = lexer->inbuffer[7];
      uint8_t idle_time = lexer->inbuffer[12];
      uint32_t receiver_status = lexer->inbuffer[20];
      receiver_status |= lexer->inbuffer[21] << 8;
      receiver_status |= lexer->inbuffer[22] << 16;
      receiver_status |= lexer->inbuffer[23] << 24;
      header_length = NOVATEL_LONG_HEADER_LENGTH;
    }
    else if ( 0x13 == lexer->inbuffer[2] ) {
      // Short header
      message_id = lexer->inbuffer[4];
      message_id |= lexer->inbuffer[5] << 8;
      message_length = lexer->inbuffer[3];
      header_length = NOVATEL_SHORT_HEADER_LENGTH;
    }
    
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
  
    /* we may need to dump the raw packet */
    GPSD_LOG(LOG_RAW, &session->context->errout,
             "NOVATEL packet type 0x%02x\n", novatel_packet->id);

    switch (message_id) {
        /* Deliver message to specific decoder based on message type */

    case NOVATEL_INSATTS:
      // INSATTS message
      GPSD_LOG(LOG_PROG, &session->context->errout, "INSATTS message\n");
      mask = insatts_message(session, &buf);
      break;

    case NOVATEL_INSSTDEVS:
      break;
    case NOVATEL_DUALANTENNAHEADING:
      break;
    case NOVATEL_CORRIMUS:
      break;
    case NOVATEL_HWMONITOR:
      break;
      
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

static bool novatel_probe_detect(struct gps_device_t *session)
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
static ssize_t novatel_control_send(struct gps_device_t *session,
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
               "writing novatel control type %02x\n");
   return gpsd_write(session, session->msgbuf, session->msgbuflen);
}

static void novatel_event_hook(struct gps_device_t *session, event_t event)
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
static gps_mask_t novatel_parse_input(struct gps_device_t *session)
{
    if (NOVATEL_PACKET == session->lexer.type) {
        return novatel_dispatch(session, session->lexer.outbuffer,
                                session->lexer.outbuflen);
    }
    return generic_parse_input(session);
}

static bool novatel_set_speed(struct gps_device_t *session,
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
static void novatel_set_mode(struct gps_device_t *session, int mode)
{
    if (mode == MODE_NMEA) {
        /* send a mode switch control string */
    } else {
        /* send a mode switch control string */
    }
}

static double novateltime_offset(struct gps_device_t *session)
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

static void novatel_wrapup(struct gps_device_t *session)
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
const struct gps_type_t driver_novatel_binary = {
    /* Full name of type */
    .type_name        = "_proto",
    /* Associated lexer packet type */
    .packet_type      = NOVATEL_PACKET,
    /* Driver tyoe flags */
    .flags            = DRIVER_NOFLAGS,
    /* Response string that identifies device (not active) */
    .trigger          = NULL,
    /* Number of satellite channels supported by the device */
    .channels         = 12,
    /* Startup-time device detector */
    .probe_detect     = novatel_probe_detect,
    /* Packet getter (using default routine) */
    .get_packet       = packet_get1,
    /* Parse message packets */
    .parse_packet     = novatel_parse_input,
    /* RTCM handler (using default routine) */
    .rtcm_writer      = pass_rtcm,
    /* non-perturbing initial query (e.g. for version) */
    .init_query        = NULL,
    /* fire on various lifetime events */
    .event_hook       = novatel_event_hook,
    /* Speed (baudrate) switch */
    .speed_switcher   = novatel_set_speed,
    /* Switch to NMEA mode */
    .mode_switcher    = novatel_set_mode,
    /* Message delivery rate switcher (not active) */
    .rate_switcher    = NULL,
    /* Minimum cycle time of the device */
    .min_cycle        = 1,
    /* Control string sender - should provide checksum and headers/trailer */
    .control_send   = novatel_control_send,
    .time_offset     = novateltime_offset,
/* *INDENT-ON* */
};
#endif  // defined(NOVATEL_ENABLE)

// vim: set expandtab shiftwidth4=
