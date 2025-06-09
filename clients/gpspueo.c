/*
 * gpspueo: Output data from IMUs to terminal so we know they look good a gpsd and print to terminal
 *
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <assert.h>
#include <errno.h>
#include <libgen.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>    // for umask()
#include <sys/stat.h>     // for umask()
#include <time.h>
#include <unistd.h>

#ifdef HAVE_GETOPT_LONG
       #include <getopt.h>
#endif

#include "../include/compiler.h"
#include "../include/gps.h"
#include "../include/gpsdclient.h"
#include "../include/os_compat.h"
#include "../include/timespec.h"

static char *progname;
static struct fixsource_t source;

// timespec_t between measurement sets
static timespec_t sample_interval_ts = {30, 0};
// milli-seconds between measurement sets
static unsigned  sample_interval_ms = 30000;

#define DEBUG_QUIET 0
#define DEBUG_INFO 1
#define DEBUG_PROG 2
#define DEBUG_RAW 3
static int debug = DEBUG_INFO;               // debug level

static struct gps_data_t gpsdata;


/* print_raw()
 * print one observation to stdout
 */
static void print_raw(struct gps_data_t *gpsdata)
{
  char tmstr[40];              // time: yyyymmdd hhmmss UTC
  struct tm *report_time;
  struct tm tm_buf;
  report_time = gmtime_r(gpsdata->attitude.mtime.tv_sec, &tm_buf);
  (void)strftime(tmstr, sizeof(tmstr), "%Y%m%d %H%M%S UTC", report_time);

  printf(" --- IMU measurement ---\n"
	 ">>>> Time %d\n"
	 ">>>> Heading %f Pitch %f Roll %f (deg)\n"
	 ">>>> Heading STD %f Pitch STD %f Roll STD %f (deg)\n"
	 ">>>> Latitude %f Longitude %f (deg) Altitude %f (m)\n"
	 ">>>> Horizontal err (m) %f Vertical error (m) %f\n"
	 ">>>> Gyro (x,y,z) (%f, %f, %f) (deg)\n"
	 ">>>> Accel (x,y,z) (%f, %f, %f) (m/s/s)\n"
	 ">>>> Raw GNSS\n"
	 "     Lat %f Lon %f (deg) Alt %f (m)"
	 "     Heading %f STD %f Tilt %f STD %f (deg)\n"
	 ">>>> System temp %f deg C\n"
	 ">>>> Gyro temps %f %f %f deg C\n"
	 ">>>> Accel temps %f %f %f deg C\n",
	 tmstr,
	 gpsdata->attitude.heading, gpsdata->attitude.pitch, gpsdata->attitude.roll,
	 gpsdata->attitude.heading_std, gpsdata->attitude.pitch_std, gpsdata->attitude.roll_std, 
	 gpsdata->fix.latitude, gpsdata->fix.longitude, gpsdata->fix.altHAE,
	 gpsdata->fix.eph, gpsdata->fix.epv,
	 gpsdata->attitude.gyro_x, gpsdata->attitude.gyro_y, gpsdata->attitude.gyro_z, 
	 gpsdata->attitude.acc_x, gpsdata->attitude.acc_y, gpsdata->attitude.acc_z,
	 gpsdata->attitude.raw_gnss.latitude, gpsdata->attitude.raw_gnss.longitude,
	 gpsdata->attitude.raw_gnss.heading, gpsdata->attitude.raw_gnss.heading_std, gpsdata->attitude.raw_gnss.tilt, gpsdata->attitude.raw_gnss.tilt_std,
	 gpsdata->attitude.temp,
	 gpsdata->attitude.gyro_temps[0], gpsdata->attitude.gyro_temps[1], gpsdata->attitude.gyro_temps[2],
  	 gpsdata->attitude.acc_temps[0], gpsdata->attitude.acc_temps[1], gpsdata->attitude.acc_temps[2]); 

}

static int sig_flag = 0;

static void quit_handler(int signum)
{
    // CWE-479: Signal Handler Use of a Non-reentrant Function
    // See: The C Standard, 7.14.1.1, paragraph 5 [ISO/IEC 9899:2011]
    // Can't log in a signal handler.  Can't even call exit().
    sig_flag = signum;
    return;
}

/* usage()
 * print usages, and exit
 */
static void usage(void)
{
    (void)fprintf(stderr,
          "Usage: %s [OPTIONS] [server[:port:[device]]]\n"
          "\n"
          "Mandatory arguments to long options are mandatory for "
          "short options too.\n"
          "     -D, --debug LVL            Set debug level, default 0\n"
	  "     -F INFILE, --filein INFILE Read from INFILE, not gpsd\n"
          "     -h, --help                 print this usage and exit\n"
          "     -i SEC, --interval SEC     Time between samples in seconds\n"
          "                                default: %0.3f\n"
          "     -n COUNT, --count COUNT    Number samples to collect\n"
          "                                default: %d\n"
          "     -V, --version              print version and exit\n"
          "defaults to '%s -n %d -i %0.3f localhost:2947'\n",
		  progname);
    exit(EXIT_FAILURE);
}

// defines for getopt_long()
#define AGENCY 301
#define ANT_E 302
#define ANT_H 303
#define ANT_N 304
#define ANT_NUM 305
#define ANT_TYPE 306
#define MARKER_NAME 307
#define MARKER_TYPE 308
#define OBSERVER 309
#define REC_NUM 310
#define REC_TYPE 311
#define REC_VERS 312


/*
 *
 * Main
 *
 */
int main(int argc, char **argv)
{
    char tmstr[40];              // time: YYYYDDDMMHH
    char tmp_fname[32];          // temp file name, for mkstemp
    int tmp_file_desc;           // temp file descriptor
    struct tm *report_time;
    struct tm tm_buf;            // temp buffer for gmtime_r()
    unsigned int flags = WATCH_ENABLE;
    char   *file_in = NULL;
    int timeout = 10;
    double f;

    progname = argv[0];

    while (1) {
        int ch;
        const char *optstring = "?D:f:F:hi:n:V";

#ifdef HAVE_GETOPT_LONG
        int option_index = 0;
        static struct option long_options[] = {
            {"debug", required_argument, NULL, 'D' },
            {"filein", required_argument, NULL, 'F' },
	    {"help", no_argument, NULL, 'h' },
            {"version", no_argument, NULL, 'V' },
            {NULL, 0, NULL, 0},
        };

        ch = getopt_long(argc, argv, optstring, long_options, &option_index);
#else
        ch = getopt(argc, argv, optstring);
#endif
        if (ch == -1) {
            break;
        }

        switch (ch) {
        case 'D':
            debug = atoi(optarg);
            break;
        case 'F':       // input file name.
            if (NULL != file_in) {
                free(file_in);
            }
            file_in = strdup(optarg);
            break;
        case 'i':               // set sampling interval
            f = safe_atof(optarg); // still in seconds
            if (3600.0 <= f) {
                (void)fprintf(stderr,
                              "WARNING: sample interval is an hour or more!\n");
            }
            sample_interval_ms = (unsigned)(1000 * f); // now in ms
            if (0 == sample_interval_ms) {
                // underflow
                sample_interval_ms = 1;
            }
            MSTOTS(&sample_interval_ts, sample_interval_ms);
            break;
        case 'V':
            (void)fprintf(stderr, "%s: version %s (revision %s)\n",
                          progname, VERSION, REVISION);
            if (NULL != file_in) {
                free(file_in);
	    }
            exit(EXIT_SUCCESS);
        case '?':
            FALLTHROUGH
        case 'h':
            FALLTHROUGH
        default:
            if (NULL != file_in) {
                free(file_in);
            }
            usage();
            // NOTREACHED
        }
    }

    // init source defaults
    memset(&source, 0, sizeof(source));
    source.server = (char *)"localhost";
    source.port = (char *)DEFAULT_GPSD_PORT;

    if (NULL != file_in) {
        // read from file, not a gpsd
        source.server = GPSD_LOCAL_FILE;
        source.port = file_in;
    } else if (optind < argc) {
        // in this case, switch to the method "socket" always
        gpsd_source_spec(argv[optind], &source);
    }
    if (DEBUG_INFO <= debug) {
        const char *device;
        if (NULL == source.device) {
            device = "Default";
        } else {
            device = source.device;
        }
        (void)fprintf(stderr, "INFO: server: %s port: %s  device: %s\n",
                      source.server, source.port, device);
    }

    // catch all interesting signals
    (void)signal(SIGTERM, quit_handler);
    (void)signal(SIGQUIT, quit_handler);
    (void)signal(SIGINT, quit_handler);

    if (0 > gps_open(source.server, source.port, &gpsdata)) {
        (void)fprintf(stderr,
                      "%s: no gpsd running or network error: %d, %s(%d)\n",
                      progname, errno, gps_errstr(errno), errno);
        exit(EXIT_FAILURE);
    }
    if (NULL != source.device) {
        flags |= WATCH_DEVICE;
    }
    (void)gps_stream(&gpsdata, flags, source.device);

    for (;;) {
        if (0 != sig_flag) {
            break;
        }
        // wait for gpsd
        if (!gps_waiting(&gpsdata, timeout * 1000000)) {
            syslog(LOG_INFO, "timeout;");
            break;
        }
        if (0 != sig_flag) {
            break;
        }
        (void)gps_read(&gpsdata, NULL, 0);
        if (ERROR_SET & gpsdata.set) {
            syslog(LOG_INFO, "gps_read() error '%s'\n", gpsdata.error);
            // dont exit, maybe useable data, maybe just EOF
            break;
        }
        if (0 != sig_flag) {
            break;
        }
        print_raw(&gpsdata);
    }

    if (0 != sig_flag &&
        SIGINT != sig_flag) {
        syslog(LOG_INFO, "exiting, signal %d received", sig_flag);
    }
    if (NULL != file_in) {
        free(file_in);      // pacify -Wanalyzer-malloc-leak
    }
    exit(EXIT_SUCCESS);
}
