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


static gps_mask_t novatel_raw_sensors(struct gps_device_t *session, an_packet_t* an_packet) {
  gps_mask_t mask = 0;

  raw_sensors_packet_t raw_sensors_packet;
  
  if (decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0) {
    session->gpsdata.attitude.gyro_x = raw_sensors_packet.gyroscopes[0]*RAD_2_DEG;
    session->gpsdata.attitude.gyro_y = raw_sensors_packet.gyroscopes[1]*RAD_2_DEG;
    session->gpsdata.attitude.gyro_z = raw_sensors_packet.gyroscopes[2]*RAD_2_DEG;

    session->gpsdata.attitude.acc_x = raw_sensors_packet.accelerometers[0];
    session->gpsdata.attitude.acc_y = raw_sensors_packet.accelerometers[1];
    session->gpsdata.attitude.acc_z = raw_sensors_packet.accelerometers[2];

    // According to SDK, but not documentation -- so not sure if this actually is true
    //session->gpsdata.attitude.mag_x = raw_sensors_packet.magnetometers[0];
    //session->gpsdata.attitude.mag_y = raw_sensors_packet.magnetometers[1];
    //session->gpsdata.attitude.mag_z = raw_sensors_packet.magnetometers[2];

    session->driver.novatel.pressure = raw_sensors_packet.pressure;
    
    session->driver.novatel.imu_temperature = raw_sensors_packet.imu_temperature;
    session->driver.novatel.pressure_temperature = raw_sensors_packet.pressure_temperature;
   
    mask |= IMU_SET;
    
    GPSD_LOG(LOG_PROG, &session->context->errout,
	     "NOVATEL: Raw sensors: gyros %.3f %.3f %.3f"
	     " accels %.3f %.3f %.3f"
	     " pressure %.3f IMU_temp %.3f pressure_temp %.3f\n",
	     session->gpsdata.attitude.gyro_x, session->gpsdata.attitude.gyro_y, session->gpsdata.attitude.gyro_z,
	     session->gpsdata.attitude.acc_x, session->gpsdata.attitude.acc_y, session->gpsdata.attitude.acc_z,
	     session->driver.novatel.pressure,
	     session->driver.novatel.imu_temperature,
	     session->driver.novatel.pressure, temperature); 
  }
  else {
    GPSD_LOG(LOG_WARN, &session->context->errout,
	     "NOVATEL: Raw sensors: unable to decode");
    return 0;
  }
  
  return mask;  
}

static gps_mask_t novatel_raw_gnss(struct gps_device_t *session, an_packet_t* an_packet) {
  gps_mask_t mask = 0;

  raw_gnss_packet_t raw_gnss_packet;
  
  if (decode_raw_gnss_packet(&raw_gnss_packet, an_packet) == 0) {

    double T = raw_gnss_packet.unix_time_seconds + (raw_gnss_packet.microseconds/1000000.0);    
    //TODO properly merge this with gpsd
    
    session->gpsdata.attitude.raw_gnss.latitude = raw_gnss_packet->position[0]*RAD_2_DEG;
    session->gpsdata.attitude.raw_gnss.longitude = raw_gnss_packet->position[1]*RAD_2_DEG;
    session->gpsdata.attitude.raw_gnss.altitude = raw_gnss_packet->position[2];  

    session->gpsdata.attitude.raw_gnss.velN = raw_gnss_packet->velocity[0];
    session->gpsdata.attitude.raw_gnss.velE = raw_gnss_packet->velocity[1];
    session->gpsdata.attitude.raw_gnss.velD = raw_gnss_packet->velocity[2];

    session->gpsdata.attitude.raw_gnss.latitude_std = raw_gnss_packet->position_standard_deviation[0];
    session->gpsdata.attitude.raw_gnss.longitude_std = raw_gnss_packet->position_standard_deviation[1];
    session->gpsdata.attitude.raw_gnss.altitude_std = raw_gnss_packet->position_standard_deviation[2];

    session->gpsdata.attitude.raw_gnss.tilt = raw_gnss_packet->tilt;
    session->gpsdata.attitude.raw_gnss.tilt_std = raw_gnss_packet->tilt_standard_deviation;
    session->gpsdata.attitude.raw_gnss.heading = raw_gnss_packet->heading;
    session->gpsdata.attitude.raw_gnss.heading_std = raw_gnss_packet->heading_standard_deviation;

    session->gpsdata.attitude.raw_gnss.flags.r = raw_gnss_packet->flags;
    
    GPSD_LOG(LOG_PROG, &session->context->errout,
	     "NOVATEL: Raw GNSS: Time %.2f"
	     " -- lat %.5f lon %.5f alt %.5f"
	     " -- STDs lat %.5f lon %.5f alt %.5f"
	     " -- Velocity N %.3f E %.3f D %.3f"
	     " -- Tilt %.3f STD %.3f"
	     " -- Heading %.3f STD %.3f"
	     " -- Flags: Fix type %u"
	     " --        Velocity valid %u"
	     " --        Time valid %u"
	     " --        External GNSS %u"
	     " --        Tilt valid %u"
	     " --        Heading valid %u",
	     session->gpsdata.attitude.raw_gnss.latitude,
	     session->gpsdata.attitude.raw_gnss.longitude,
	     session->gpsdata.attitude.raw_gnss.altitude,
	     session->gpsdata.attitude.raw_gnss.latitude_std,
	     session->gpsdata.attitude.raw_gnss.longitude_std,
	     session->gpsdata.attitude.raw_gnss.altitude_std,
	     session->gpsdata.attitude.raw_gnss.velN,
	     session->gpsdata.attitude.raw_gnss.velE,
	     session->gpsdata.attitude.raw_gnss.velD,
	     session->gpsdata.attitude.raw_gnss.tilt, session->gpsdata.attitude.raw_gnss.tilt_std,
	     session->gpsdata.attitude.raw_gnss.heading, heading_std,
	     session->gpsdata.attitude.raw_gnss.flags.b.fix_type,
	     session->gpsdata.attitude.raw_gnss.flags.b.velocity_valid,
	     session->gpsdata.attitude.raw_gnss.flags.b.time_valid,
	     session->gpsdata.attitude.raw_gnss.flags.b.external_gnss,
	     session->gpsdata.attitude.raw_gnss.flags.b.tilt_valid,
	     session->gpsdata.attitude.raw_gnss.flags.b.heading_valid);
  }
  else {
    GPSD_LOG(LOG_WARN, &session->context->errout,
	     "NOVATEL: Raw GNSS: unable to decode");
    return 0;
  }
  
  return mask;  
}

static gps_mask_t novatel_geodetic_position(struct gps_device_t *session, an_packet_t* an_packet) {
  gps_mask_t mask = 0;

  geodetic_position_packet_t geodetic_position_packet;
  
  if (decode_geodetic_position_packet(&geodetic_position_packet, an_packet) == 0) {
    session->newdata.latitude = geodetic_position_packet->position[0]*RAD_2_DEG;
    session->newdata.longitude = geodetic_position_packet->position[1]*RAD_2_DEG;
    session->newdata.altHAE = geodetic_position_packet->position[2]; // Height above ellipsoid in meters
    
    mask |= LATLON_SET;
    mask |= ALTITUDE_SET;

    GPSD_LOG(LOG_PROG, &session->context->errout,
	     "NOVATEL: Geodetic position: lat %.5f lon %.5f alt %.5f\n",
	     session->newdata.latitude,
	     session->newdata.longitude,
	     session->newdata.altHAE);
  }
  else {
    GPSD_LOG(LOG_WARN, &session->context->errout,
	     "NOVATEL: Geodetic position: unable to decode");
    return 0;
  }
  
  return mask;  
}


static gps_mask_t novatel_ned_velocity(struct gps_device_t *session, an_packet_t* an_packet) {
  gps_mask_t mask = 0;

  ned_velocity_packet_t euler_orientation_packet;
  // Decode the packet
  // GPSD expects attitude in degrees
  
  if (decode_ned_velocity_packet(&ned_velocity_packet, an_packet) == 0) {
    session->newdata.NED.velN = ned_velocity_packet->velocity[0];
    session->newdata.NED.velE = ned_velocity_packet->velocity[1];
    session->newdata.NED.velD = ned_velocity_packet->velocity[2];

    GPSD_LOG(LOG_PROG, &session->context->errout,
	     "NOVATEL: NED Velocity: %.5f %.5f %.5f\n",
	     session->newdata.NED.velN,
	     session->newdata.NED.velE,
	     session->newdata.NED.velD);
  }
  else {
    GPSD_LOG(LOG_WARN, &session->context->errout,
	     "NOVATEL: NED Velocity: unable to decode");
    return 0;
  }
  
  return mask;  
}
/*
 * Novatel INS Attitude message with short header (
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

static gps_mask_t novatel_system_temperature(struct gps_device_t *session, an_packet_t* an_packet) {
  gps_mask_t mask = 0;

  system_temperature_packet_t system_temperature_packet;
  
  if (decode_system_temperature_packet(&system_temperature_packet, an_packet) == 0) {
   
    session->driver.novatel.system_temperature = system_temperature_packet->system_temperature;

    GPSD_LOG(LOG_PROG, &session->context->errout,
	     "NOVATEL: System temperature (deg C): %.1f\n",
	     session->driver.novatel.system_temperature);
  }
  else {
    GPSD_LOG(LOG_WARN, &session->context->errout,
	     "NOVATEL: System temperature: unable to decode");
    return 0;
  }
  
  return mask;  
}

/*
 * Decode the navigation solution message
 */
static gps_mask_t novatel_msg_navsol(struct gps_device_t *session,
                                     unsigned char *buf, size_t data_len)
{
    gps_mask_t mask;
    int flags;
    double Px, Py, Pz, Vx, Vy, Vz;

    if (data_len != NOVATEL_NAVSOL_MSG_LEN)
        return 0;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "novatel NAVSOL - navigation data\n");
    /* if this protocol has a way to test message validity, use it */
    flags = GET_FLAGS();
    if ((flags & NOVATEL_SOLUTION_VALID) == 0)
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
static gps_mask_t novatel_msg_utctime(struct gps_device_t *session,
                                      unsigned char *buf, size_t data_len)
{
    double t;

    if (data_len != UTCTIME_MSG_LEN)
        return 0;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "novatel UTCTIME - navigation data\n");
    /* if this protocol has a way to test message validity, use it */
    flags = GET_FLAGS();
    if ((flags & NOVATEL_TIME_VALID) == 0)
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
static gps_mask_t novatel_msg_svinfo(struct gps_device_t *session,
                                     unsigned char *buf, size_t data_len)
{
    unsigned char i, st, nchan, nsv;
    unsigned int tow;

    if (data_len != SVINFO_MSG_LEN )
        return 0;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "novatel SVINFO - navigation data\n");
    /* if this protocol has a way to test message validity, use it */
    flags = GET_FLAGS();
    if ((flags & NOVATEL_SVINFO_VALID) == 0)
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
static gps_mask_t novatel_msg_raw(struct gps_device_t *session,
                                  unsigned char *buf, size_t data_len)
{
    unsigned char i, st, nchan, nsv;
    unsigned int tow;

    if (data_len != RAW_MSG_LEN )
        return 0;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "novatel RAW - raw measurements\n");
    /* if this protocol has a way to test message validity, use it */
    flags = GET_FLAGS();
    if ((flags & NOVATEL_SVINFO_VALID) == 0)
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
