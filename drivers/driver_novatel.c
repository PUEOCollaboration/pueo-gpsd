/*
  * Novatel format driver
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  /* must be before all includes */

#include "../include/gpsd.h"
#include "../include/driver_novatel.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#if defined(NOVATEL_ENABLE)

#include "../include/bits.h"

static  gps_mask_t novatel_parse_input(struct gps_device_t *);
static  gps_mask_t novatel_dispatch(struct gps_device_t *, unsigned char *,
                                    size_t );


// Message about status of housekeeping reading
static struct vlist_t novatel_hwstatus_str[] = {
  {0, "Acceptable"},
  {1, "Warning! Low"},
  {2, "Error!! Low"},
  {3, "Warning! High"},
  {4, "Error!! High"},
  {0, NULL},
};

// Name of housekeeping reading
static struct vlist_t novatel_hwstatus_type[] = {
  {0x01, "Temperature (C)"},
  {0x02, "Antenna Current (A)"},
  {0x06, "Digital Core 3V3 Voltage (V)"},
  {0x07, "Antenna Voltage (V)"},
  {0x08, "Digital 1V2 Core Voltage (V)"},
  {0x0F, "Regulated Supply Voltage (V)"},
  {0x11, "1V8"},
  {0x15, "5V Voltage (V)"},
  {0x16, "Secondary Temperature (C)"},
  {0x17, "Peripheral Core Voltage (V)"},
  {0x18, "Secondary Antenna Current (A)"},
  {0x19, "Secondary Antenna Voltage (V)"},
  {0, NULL},
};

/*
 * These methods may be called elsewhere in gpsd
 */
static  ssize_t novatel_control_send(struct gps_device_t *, char *, size_t);
static  void novatel_event_hook(struct gps_device_t *, event_t);
//static  bool novatel_set_speed(struct gps_device_t *, speed_t, char, int);
//static  void novatel_set_mode(struct gps_device_t *, int);

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
static gps_mask_t insatts_message(struct gps_device_t *session, unsigned char *buf) {
  gps_mask_t mask = 0;
  
  unsigned long week = getleu32(buf, NOVATEL_SHORT_HEADER_LENGTH);
  timespec_t seconds_into_week;
  DTOTS(&seconds_into_week, getled64((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+4));
  TS_NORM(&seconds_into_week);
  session->newdata.time = gpsd_gpstime_resolv(session, week, seconds_into_week);
  mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
  
  session->gpsdata.attitude.roll = getled64((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+12);
  session->gpsdata.attitude.pitch = getled64((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+20);
  session->gpsdata.attitude.heading = getled64((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+28);
  mask |= ATTITUDE_SET;

  // Status at H+36, 4 bytes
  
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
static gps_mask_t insstdevs_message(struct gps_device_t *session, unsigned char *buf) {
  gps_mask_t mask = 0;

    // Get GNSS time from header
  unsigned short week = getleu16(buf, 6);
  timespec_t seconds_into_week;
  unsigned long milliseconds_into_week = getleu32(buf, 8);
  DTOTS(&seconds_into_week, milliseconds_into_week/1000.0);
  TS_NORM(&seconds_into_week);
  session->newdata.time = gpsd_gpstime_resolv(session, week, seconds_into_week);
  mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;

  float latitude_std = getlef32((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH);
  float longitude_std = getlef32((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+4);
  
  session->gpsdata.dop.xdop = latitude_std;
  session->gpsdata.dop.ydop = longitude_std;
  mask |= DOP_SET;
    
  session->newdata.eph = latitude_std > longitude_std ? latitude_std : longitude_std; 
  session->newdata.epv = getlef32((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+8); // Height above ellipsoid in meters
    
  mask |= HERR_SET;
  mask |= VERR_SET;

  GPSD_LOG(LOG_PROG, &session->context->errout,
	   "NOVATEL: Position standard deviation: eph %.3f epv %.3f\n",
	   session->newdata.eph,
	   session->newdata.epv);

  
  session->gpsdata.attitude.roll_std = getlef32((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+24);
  session->gpsdata.attitude.pitch_std = getlef32((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+28);
  session->gpsdata.attitude.heading_std = getlef32((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+32);
  mask |= ATTITUDE_SET;

  //Status at H+36, 4 bytes

  GPSD_LOG(LOG_PROG, &session->context->errout,
	   "NOVATEL: Euler attitude standard deviation: roll %.5f pitch %.5f heading %.5f\n",
	   session->gpsdata.attitude.roll_std,
	   session->gpsdata.attitude.pitch_std,
	   session->gpsdata.attitude.heading_std);
  
  return mask;  
}

/*
 * Novatel message -- position derived from the best source (INS, or GNSS)
 * BESTPOS
 */
static gps_mask_t bestpos_message(struct gps_device_t *session, unsigned char *buf) {
  gps_mask_t mask = 0;

  // Time from header
  unsigned short week = getleu16(buf, 14);
  timespec_t seconds_into_week;
  unsigned long milliseconds_into_week = getleu32(buf, 16);
  DTOTS(&seconds_into_week, milliseconds_into_week/1000.0);
  TS_NORM(&seconds_into_week);
  session->newdata.time = gpsd_gpstime_resolv(session, week, seconds_into_week);
  mask = TIME_SET | NTPTIME_IS | GOODTIME_IS;

  // Statuses
  unsigned int solution_status = getleu32(buf, NOVATEL_LONG_HEADER_LENGTH);
  unsigned int position_status = getleu32(buf, NOVATEL_LONG_HEADER_LENGTH+4);
  switch (solution_status) {
  case 0:
    // SOL_COMPUTED
    switch (position_status) {
    case 2:
      // FIXEDHEIGHT -- no altitude yet
      session->newdata.mode = MODE_2D;
      session->newdata.status = STATUS_GPS;
      break;
    case 16:
      //SINGLE -- solution only from GNSS
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_GPS;
      break;
    case 17:
      // PSRDIFF -- differential corrections
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_DGPS;
      break;
    case 18:
      // WAAS -- SBAS corrections
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_GPS;
      break;
    case 19:
      // PROPAGATED -- propagated by Kalman filter without observations; aka Dead Reckoning?
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_DR;
      break;
    case 32:
      // L1_FLOAT -- Single frequency RTK with float carrier phase ambiguities
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_RTK_FLT;
      break;
    case 34:
      // NARROW_FLOAT -- Multi-frequency RTK with float carrier phase ambiguities
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_RTK_FLT;
      break;
    case 48:
      // L1_INT -- Single-frequency RTK with integer ambiguities
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_RTK_FIX;
      break;
    case 49:
      // WIDE_INT -- Multi-frequency RTK solution with wide-lane integer ambiguities
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_RTK_FIX;
      break;
    case 50:
      // NARROW_INT -- Multi-frequency RTK solution with narrow-lane integer ambiguities
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_RTK_FIX;
      break;
    case 52:
      // INS_SBAS -- INS with SBAS correction
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_GPS;
      break;
    case 53:
      // INS_PSRSP -- INS + SINGLE
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_GPS;
      break;
    case 54:
      // INS_PSRDIFF -- INS + differential corrections
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_DGPS;
      break;
    case 55:
      // INS_RTKFLOAT -- INS + L1_FLOAT or NARROW_FLOAT
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_RTK_FLT;
      break;
    case 56:
      // INS_RTKFIXED -- INS + L1_INT, WIDE_INT, or NARROW_INT)
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_RTK_FIX;
      break;
    case 69:
      // PPP
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_RTK_FLT;
      break;
    case 74:
      // INS_PPP
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_GPS;
      break;
    case 78:
      // PPP_BASIC
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_GPS;
      break;
    case 80:
      // INS_PPP_BASIC
      session->newdata.mode = MODE_3D;
      session->newdata.status = STATUS_GPS;
      break;
    case 0:
    default:
      // no fix
      session->newdata.mode = MODE_NO_FIX;
      session->newdata.status = STATUS_UNK;
    }
    break;
  default:
    // Everything else, no solution yet (for various reasons...)
    session->newdata.mode = MODE_NO_FIX;
    session->newdata.status = STATUS_UNK;
  }
    
  mask |= MODE_SET | STATUS_SET; 
  
  session->newdata.latitude = getled64((const char *)buf, NOVATEL_LONG_HEADER_LENGTH+8);
  session->newdata.longitude = getled64((const char *)buf, NOVATEL_LONG_HEADER_LENGTH+16);
  session->newdata.altMSL = getled64((const char *)buf, NOVATEL_LONG_HEADER_LENGTH+24); // height above mean sea level (meters)

  float latitude_std = getlef32((const char *)buf, NOVATEL_LONG_HEADER_LENGTH+40);
  float longitude_std = getlef32((const char *)buf, NOVATEL_LONG_HEADER_LENGTH+44);
  
  session->gpsdata.dop.xdop = latitude_std;
  session->gpsdata.dop.ydop = longitude_std;
  mask |= DOP_SET;
    
  session->newdata.eph = latitude_std > longitude_std ? latitude_std : longitude_std; 
  session->newdata.epv = getlef32((const char *)buf, NOVATEL_LONG_HEADER_LENGTH+48); // Height std (m)
    
  mask |= HERR_SET;
  mask |= VERR_SET;

  session->gpsdata.satellites_visible = (int)buf[NOVATEL_LONG_HEADER_LENGTH + 64];
  session->gpsdata.satellites_used = (int)buf[NOVATEL_LONG_HEADER_LENGTH + 65];
  mask |= SATELLITE_SET | USED_IS;

  mask |= ONLINE_SET | LATLON_SET | ALTITUDE_SET;
    
    
  GPSD_LOG(LOG_PROG, &session->context->errout,
	   "NOVATEL: Best Pos: lat %.5f lon %.5f alt %.5f\n"
	   "         %d satellites visible  %d satellites used",
	   session->newdata.latitude, session->newdata.longitude, session->newdata.altMSL,
	   session->gpsdata.satellites_visible, session->gpsdata.satellites_used);

  return mask;
}


/*
 * Novatel message -- Heading derived from the vector between the two antennas
 * DUALANTENNAHEADING
 */
static gps_mask_t dualantennaheading_message(struct gps_device_t *session, unsigned char *buf) {
  gps_mask_t mask = 0;

  // Solution status at H, 4 bytes
  // Position type at H+4, 4 bytes

  // baseline length, probably unecessary, at H+8, 4 bytes
 
  session->newdata.dualantenna.heading = getlef32((const char *)buf, NOVATEL_LONG_HEADER_LENGTH+12);
  session->newdata.dualantenna.tilt = getlef32((const char *)buf, NOVATEL_LONG_HEADER_LENGTH+16);
  session->newdata.dualantenna.heading_std = getlef32((const char *)buf, NOVATEL_LONG_HEADER_LENGTH+24);
  session->newdata.dualantenna.tilt_std = getlef32((const char *)buf, NOVATEL_LONG_HEADER_LENGTH+28);

    
  GPSD_LOG(LOG_PROG, &session->context->errout,
	   "NOVATEL: Dual Antenna Heading"
	   " -- Heading %.3f STD %.3f"
	   " -- Tilt %.3f STD %.3f\n",
	   session->newdata.dualantenna.heading, session->newdata.dualantenna.heading_std,
	   session->newdata.dualantenna.tilt, session->newdata.dualantenna.tilt_std
	   );

    
  return mask;
}

/*
 * Novatel message -- raw IMU data, corrected for gravity, earth rotation, sensor errors with short header
 * CORRIMUS
 */
static gps_mask_t corrimus_message(struct gps_device_t *session, unsigned char *buf) {
  gps_mask_t mask = 0;

  // Get GNSS time from header
  unsigned short week = getleu16(buf, 6);
  timespec_t seconds_into_week;
  unsigned long milliseconds_into_week = getleu32(buf, 8);
  DTOTS(&seconds_into_week, milliseconds_into_week/1000.0);
  TS_NORM(&seconds_into_week);
  session->newdata.time = gpsd_gpstime_resolv(session, week, seconds_into_week);
  mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;

  // Plan is to output at 20Hz -- this goes into rate calculation
  float data_rate = 20;
  
  // IMU samples used
  unsigned long imudatacount = getleu32(buf, NOVATEL_SHORT_HEADER_LENGTH);
  
  session->gpsdata.attitude.gyro_x = getled64((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+4)*RAD_2_DEG*(data_rate/imudatacount);
  session->gpsdata.attitude.gyro_y = getled64((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+12)*RAD_2_DEG*(data_rate/imudatacount);
  session->gpsdata.attitude.gyro_z = getled64((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+20)*RAD_2_DEG*(data_rate/imudatacount);
  
  session->gpsdata.attitude.acc_x = getled64((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+28)*(data_rate/imudatacount);
  session->gpsdata.attitude.acc_y = getled64((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+36)*(data_rate/imudatacount);
  session->gpsdata.attitude.acc_z = getled64((const char *)buf, NOVATEL_SHORT_HEADER_LENGTH+44)*(data_rate/imudatacount);

  mask |= ATTITUDE_SET;
    
  GPSD_LOG(LOG_PROG, &session->context->errout,
	   "NOVATEL: IMU data:"
	   " gyros (deg/s) %.3f %.3f %.3f"
	   " accels (m/s/s) %.3f %.3f %.3f",
	   session->gpsdata.attitude.gyro_x, session->gpsdata.attitude.gyro_y, session->gpsdata.attitude.gyro_z,
	   session->gpsdata.attitude.acc_x, session->gpsdata.attitude.acc_y, session->gpsdata.attitude.acc_z);

    
  return mask;
}

/*
 * Novatel message -- Temperatures, voltages, etc.
 * HWMONITOR
 */
static gps_mask_t hwmonitor_message(struct gps_device_t *session, unsigned char *buf) {
  gps_mask_t mask = 0;
  unsigned long num_measurements = getleu32(buf, NOVATEL_LONG_HEADER_LENGTH);

  GPSD_LOG(LOG_PROG, &session->context->errout,
	   "NOVATEL: Housekeeping: ");

  for (unsigned i=0; i<num_measurements; i++){
    float reading = getlef32((const char *)buf, NOVATEL_LONG_HEADER_LENGTH+4+8*i);
    unsigned int status = getub(buf, NOVATEL_LONG_HEADER_LENGTH+8+8*i);
    unsigned int type = getub(buf, NOVATEL_LONG_HEADER_LENGTH+9+8*i);

    if ( 0 == strcmp(val2str(type, novatel_hwstatus_type), "Temperature (C)") ){
      session->gpsdata.attitude.temp = reading;
    }
    else if ( 0 == strcmp(val2str(type, novatel_hwstatus_type), "Antenna Current (A)") ){
      session->gpsdata.attitude.antenna1_current;
    }
    else if ( 0 == strcmp(val2str(type, novatel_hwstatus_type), "Secondary Antenna Current (A)") ){
      session->gpsdata.attitude.antenna2_current;
    }
    
    GPSD_LOG(LOG_PROG, &session->context->errout,
	     "  %s %.1f -- %s\n",
	     val2str(type, novatel_hwstatus_type), reading, val2str(status, novatel_hwstatus_str));
  }
  mask |= ATTITUDE_SET;

  return mask;
}

/**
 * Parse the data from the device
 */
gps_mask_t novatel_dispatch(struct gps_device_t *session,
                            unsigned char *buf, size_t len)
{
//    size_t i;
//    int used, visible, retmask = 0;

    gps_mask_t mask = 0;

    uint8_t header_length = 0;
    uint16_t message_length = 0;
    uint16_t message_id = 0;
    if ( 0x12 == buf[2] ) {
      // Long header
      message_id = buf[4];
      message_id |= buf[5] << 8;
      message_length = buf[8];
      message_length |= buf[9] << 8;
      //uint8_t idle_time = buf[12]; // not used now, but maybe in the future
      //uint32_t receiver_status = buf[20];
      //receiver_status |= buf[21] << 8;
      //receiver_status |= buf[22] << 16;
      //receiver_status |= buf[23] << 24;
      header_length = NOVATEL_LONG_HEADER_LENGTH;
    }
    else if ( 0x13 == buf[2] ) {
      // Short header
      message_id = buf[4];
      message_id |= buf[5] << 8;
      message_length = buf[3];
      header_length = NOVATEL_SHORT_HEADER_LENGTH;
    }
    
    if (len == 0)
        return 0;
  
    /* we may need to dump the raw packet */
    GPSD_LOG(LOG_RAW, &session->context->errout,
             "NOVATEL packet type 0x%02x\n", message_id);

    switch (message_id) {
        /* Deliver message to specific interpreter based on message type */
    case NOVATEL_INSATTS:
      // INSATTS message
      GPSD_LOG(LOG_PROG, &session->context->errout, "INSATTS message\n");
      mask = insatts_message(session, buf);
      break;
    case NOVATEL_BESTPOS:
      // BESTPOS message
      GPSD_LOG(LOG_PROG, &session->context->errout, "BESTPOS message\n");
      mask = bestpos_message(session, buf);
      break;
    case NOVATEL_INSSTDEVS:
      // INSSTDEVS message
      GPSD_LOG(LOG_PROG, &session->context->errout, "INSSTDEVS message\n");
      mask = insstdevs_message(session, buf);
      break;
    case NOVATEL_DUALANTENNAHEADING:
      // DUALANTENNAHEADING message
      GPSD_LOG(LOG_PROG, &session->context->errout, "DUALANTENNAHEADING message\n");
      mask = dualantennaheading_message(session, buf);
      break;
    case NOVATEL_CORRIMUS:
      // CORRIMUS message
      GPSD_LOG(LOG_PROG, &session->context->errout, "CORRIMUS message\n");
      mask = corrimus_message(session, buf);
      break;
    case NOVATEL_HWMONITOR:
      // HWMONITOR message
      GPSD_LOG(LOG_PROG, &session->context->errout, "HWMONITOR message\n");
      mask = hwmonitor_message(session, buf);
      break;
      
    default:
      GPSD_LOG(LOG_WARN, &session->context->errout,
	       "unknown packet id %d length %d\n", message_id, message_length);
      return 0;
    }

    return mask | ONLINE_SET;

}

/**********************************************************
 *
 * Externally called routines below here
 *
 **********************************************************/

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
               "\n\nNOVATEL: writing novatel control type %s\n", session->msgbuf);
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
const struct gps_type_t driver_novatel = {
    /* Full name of type */
    .type_name        = "Novatel",
    /* Associated lexer packet type */
    .packet_type      = NOVATEL_PACKET,
    /* Driver tyoe flags */
    .flags            = DRIVER_NOFLAGS,
    /* Response string that identifies device (not active) */
    .trigger          = NULL,
    /* Number of satellite channels supported by the device */
    .channels         = 12,
    /* Startup-time device detector */
    .probe_detect     = NULL,
    /* Packet getter (using default routine) */
    .get_packet       = packet_get1,
    /* Parse message packets */
    .parse_packet     = novatel_parse_input,
    /* RTCM handler (using default routine) */
    //.rtcm_writer      = pass_rtcm,
    /* non-perturbing initial query (e.g. for version) */
    .init_query        = NULL,
    /* fire on various lifetime events */
    .event_hook       = novatel_event_hook,
    /* Speed (baudrate) switch */
    //.speed_switcher   = novatel_set_speed,
    /* Switch to NMEA mode */
    //.mode_switcher    = novatel_set_mode,
    /* Message delivery rate switcher (not active) */
    .rate_switcher    = NULL,
    /* Minimum cycle time of the device */
    .min_cycle        =  { .tv_sec = 1, .tv_nsec = 0},
    /* Control string sender - should provide checksum and headers/trailer */
    .control_send   = novatel_control_send,
    //.time_offset     = novateltime_offset,
/* *INDENT-ON* */
};
#endif  // defined(NOVATEL_ENABLE)

// vim: set expandtab shiftwidth4=
