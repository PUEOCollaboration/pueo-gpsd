#ifndef _GPSD_NOVATEL_H_
#define _GPSD_NOVATEL_H_

#define NOVATEL_SHORT_HEADER_LENGTH 12
#define NOVATEL_LONG_HEADER_LENGTH 28

typedef enum {
  // These are the ones needed by PUEO for now; more may be added later
  NOVATEL_BESTPOS = 42,
  NOVATEL_INSATTS = 319,
  NOVATEL_INSSTDEVS = 2052,
  NOVATEL_DUALANTENNAHEADING = 2042,
  NOVATEL_CORRIMUS = 2264,
  NOVATEL_HWMONITOR = 963,  
} novatel_message_t;

unsigned long CRC32Value(int i);
unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char *ucBuffer);

#endif // _GPSD_NOVATEL_H_
