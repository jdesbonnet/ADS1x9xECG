/**
 * ads1x9x_evm.c- a command line utility to communicate with the
 * TI ADS1x9x ECG/EEG AFE EVM board running the supplied firmware.
 *
 * EVM board schematics, BOM, firmware and sourcecode at
 * ftp://ftp.ti.com/pub/data_acquisition/ECG_FE/ADS1292/
 *
 * Author: Joe Desbonnet, jdesbonnet@gmail.com
 * 
 *
 * Version 0.1 (13 September 2012)
 * 
 * To compile:
 * gcc -o ads1x9x_evm ads1x9x_evm.c
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <stdarg.h>


#define APP_NAME "ads1x9x_evm"
#define VERSION "0.1, 13 Sep 2012"

#define TRUE 1
#define FALSE 0

#define FORMAT_DECIMAL 1
#define FORMAT_BINARY 2

#define FILTER_40HZ_LOWPASS 1
// 50Hz notch and 0.5-150Hz pass
#define FILTER_50HZ_NOTCH 2
// 60Hz notch and 0.5-150Hz pass
#define FILTER_60HZ_NOTCH 3

// Host/USB protocol command definitions in ADS1x9x_USB_Communication.h


#define CMD_REG_WRITE			0x91
#define CMD_REG_READ			0x92


#define CMD_DATA_STREAMING		0x93

#define CMD_ACQUIRE_DATA		0x94

#define PROC_DATA_DOWNLOAD_COMMAND	0x95
#define DATA_DOWNLOAD_COMMAND		0x96
#define FIRMWARE_UPGRADE_COMMAND	0x97
#define START_RECORDING_COMMAND		0x98


#define CMD_QUERY_FIRMWARE_VERSION		0x99

#define STATUS_INFO_REQ 			0x9A
#define CMD_FILTER_SELECT		0x9B
#define ERASE_MEMORY_COMMAND		0x9C

// Seems to have no effect
#define CMD_RESTART				0x9D

// Host <-> EVM data frames
// START_DATA_HEADER (packet type/cmd) (data ...) END_DATA_HEADER
#define START_DATA_HEADER			0x02
#define END_DATA_HEADER				0x03

// ADS1292R registers. ADS129x[R] datasheet, Table 14, page 39.
// RegAddr RegName: Bit7 Bit6 .. Bit0 [value on reset]
// 0x00 ID: REV_ID7 REV_ID6 REV_ID5 1 0 0 REV_ID1 REV_ID0 [factory programmed]
// 0x01 CONFIG1: SINGLE-SHOT 0 0 0 0 DR2 DR1 DR0 [0x02 on reset]
// 0x02 CONFIG2: 1 PBD_LOFF_COMP PDB_REFBUF VREF_4V CLK_EN 0 INT_TEST TEST_FREQ [0x80 on reset]
// 0x03 LOFF: COMP_TH2 COMP_TH1 COMP_TH0 1 ILEAD_OFF1 ILEAD_OFF0 0 FLEAD_OFF [0x10 on reset]
// 0x04 CH1SET: PD1 GAIN1_2 GAIN1_1 GAIN1_0 MUX1_3 MUX1_2 MUX1_1 MUX1_0 [0x00]
// 0x05 CH2SET: PD2 GAIN2_2 GAIN2_1 GAIN2_0 MUX2_3 MUX2_2 MUX2_1 MUX2_0 [0x00]
//
#define REG_ID 0x00


// A structure that represents one frame of Host/USB protocol.
typedef struct  {
	uint8_t type;
	uint8_t length;
	uint8_t data[128];
} ads1x9x_evm_frame_t;

// The debug level set with the -d command line switch
int debug_level = 0;

// Use -q flag to enable quiet mode. Warning messages will be suppressed.
int quiet_mode = FALSE;

// Set to true in signal_handler to signal exit from main loop
int exit_flag = FALSE;

void debug (int level, const char *msg, ...);
void warning (const char *msg, ...);

uint8_t cmd_buf[256];

/**
 * Open serial IO device to ADS1292R EVM 
 * (8N1, 57600bps, raw mode, no handshaking)
 *
 * @param deviceName Pointer to string with device name (eg "/dev/ttyACM0")
 * @return Operating system file descriptor or -1 if there was an error.
 */
int ads1x9x_evm_open(char *deviceName, int bps) {

	int fd = open(deviceName,O_RDWR);
	if (fd==0) {
		fprintf (stderr,"Error: unable to open device %s\n",deviceName);
		return -1;
	} 

	struct termios tios;
	int status=tcgetattr(fd,&tios);
	if (status < 0) {
    	fprintf (stderr,"Error: error calling tcgetattr\n");
		return -1;
	}

	int speed = B9600;
	switch (bps) {
		case 9600:
			speed = B9600;
			break;
		case 19200:
			speed = B19200;
			break;
		case 38400:
			speed = B38400;
			break;
		case 57600:
			speed = B57600;
			break;
		case 115200:
			speed = B115200;
			break;
		default:
			fprintf (stderr,"Unsupported speed %d bps\n", bps);
	}

	// Set tx/rx speed at 115200bps, and set raw mode
 	cfsetispeed(&tios,speed);
 	cfsetospeed(&tios,speed);
	cfmakeraw(&tios);

	tios.c_cflag &= ~CSTOPB; // Set 1 stop bit
	tios.c_cflag |= (CREAD | CLOCAL); // Enable receiver and disable hardware flow control
	tios.c_oflag = 0; // Disable some modem settings


	//tios.c_cc[VMIN] = 1; 
	//tios.c_cc[VTIME] = 0;

	tcsetattr(fd, TCSANOW, &tios);

	return fd;
}

/**
 * Close serial IO device.
 * @param fd File descriptor
 */
void ads1x9x_evm_close(int fd) {
	close(fd);
}


/**
 * Display to stderr current version of this application.
 */
void version () {
	fprintf (stderr,"%s, version %s\n", APP_NAME, VERSION);
}

/**
 * Display help and usage information. 
 */
void usage () {
	fprintf (stderr,"\n");
	fprintf (stderr,"Usage: ads1x9x_evm [-q] [-v] [-h] [-d level] device\n");

	//fprintf (stderr,"  -c channel \t Set channel. Allowed values: 11 to 26.\n");	
	fprintf (stderr,"\n");
	fprintf (stderr,"Options:\n");
	fprintf (stderr,"  -d level \t Set debug level, 0 = min (default), 9 = max verbosity\n");
	fprintf (stderr,"  -q \t Quiet mode: suppress warning messages.\n");
	fprintf (stderr,"  -v \t Print version to stderr and exit\n");
	fprintf (stderr,"  -h \t Display this message to stderr and exit\n");
	fprintf (stderr,"\n");
	fprintf (stderr,"Parameters:\n");
	fprintf (stderr,"  device:  the unix device file corresponding to the device (often /dev/ttyACM0)\n");
	fprintf (stderr,"\n");
	//fprintf (stderr,"See this blog post for details: \n    http://jdesbonnet.blogspot.com/2012/04/stm32w-rfckit-as-802154-network.html\n");
	fprintf (stderr,"Version: ");
	version();
	fprintf (stderr,"Author: Joe Desbonnet, jdesbonnet@gmail.com.\n");
	fprintf (stderr,"Copyright 2012. Source released under BSD licence.\n");
	fprintf (stderr,"\n");
}



/**
 * Display debug message if suitable log level is selected. 
 * Use vararg mechanism to allow use similar to the fprintf()
 * function.
 *
 * @param level Display this message if log level is greater
 * or equal this level. Otherwise ignore the message.
 * @param msg  Format string as described in fprintf()
 */
void debug (int level, const char* msg, ...) {
	if (level >= debug_level) {
		return;
	}
	va_list args;
	va_start(args, msg);		// args after 'msg' are unknown
	vfprintf(stderr, msg, args);
	fprintf(stderr,"\n");
	fflush(stderr);
	va_end(args);
}
/**
 * Display warning message if unless quiet_mode is enabled.
 * 
 * @param msg  Format string as described in fprintf()
 */
void warning (const char* msg, ...) {
	if (quiet_mode) {
		return;
	}
	fprintf(stderr,"WARNING: ");
	va_list args;
	va_start(args, msg);		// args after 'msg' are unknown
	vfprintf(stderr, msg, args);
	fprintf(stderr,"\n");
	fflush(stderr);
	va_end(args);
}

/**
 * Signal handler for handling SIGPIPE and...
 */
void signal_handler(int signum, siginfo_t *info, void *ptr) {
	debug (1, "Received signal %d originating from PID %lu\n", signum, (unsigned long)info->si_pid);
	//exit(EXIT_SUCCESS);
	exit_flag = TRUE;
}

/**
 * Continue to read from file handle fd until 'length' bytes
 * have been read.
 */
void read_n_bytes (int fd, void *buf, int length) {
	//fprintf (stderr,"read_n_bytes length=%d\n",length);
	int n=0;
	while (n < length) {
		n += read(fd,buf+n,length-n);
		//fprintf (stderr,"*");
		//fflush(stderr);
	}
}

/**
 * Display length bytes from pointer buf in zero padded
 * hex.
 */
void display_hex(uint8_t *buf, int length) {
	int i;
	for (i = 0; i < length; i++) {
		fprintf (stderr,"%02X ",buf[i]);
	}
}

/**
 * Read a Host/EVM protocol frame from ADS1x9x EVM module.
 *
 * @return TBD
 */
int ads1x9x_evm_read_frame (int fd, ads1x9x_evm_frame_t *frame) {
	
	int i,c=0;

	// Wait for start of data header
	do {
		read_n_bytes (fd,&c,1);
		//fprintf (stderr,"[ %x ] ",c);
	} while (c != START_DATA_HEADER);

	//fprintf (stderr,"*** START_OF_DATA ***\n");

	// Need to know type of frame to calculate length
	read_n_bytes (fd,&c,1);
	//fprintf (stderr,"c=%02x\n",c);

	frame->type = c;

	switch (c) {
		case CMD_DATA_STREAMING:
			// Read HR + RESP + LOFF + 14 x (ch1(16bits) + ch2(16bits))  + 2xEOH = 61 bytes
			frame->length = 59;
			read_n_bytes (fd,frame->data,61);
			break;

		case CMD_REG_READ:
		case CMD_QUERY_FIRMWARE_VERSION:
			frame->length = 5;
			read_n_bytes (fd,frame->data,5);
			break;
		
		default:
			fprintf (stderr,"unknown packet type %x\n",c);
			uint8_t buf[4];
			i = 0;
			do {
				read_n_bytes(fd,buf,1);
				display_hex(buf,1);
				i++;
			} while (buf[0] != END_DATA_HEADER);
			fprintf (stderr, "\n");
			frame->length = i-1;
	}

	return 0;
}
/**
 * @deprecated  Use ads1x9x_evm_read_frame() instead.
 *
 * @return The entire frame length (excluding cksum) if successful, -1 on error.
 */
int ads1x9x_evm_read_response (int fd) {
	
	int i;
	uint8_t c,v,heart_rate,respiration,lead_off;
	uint8_t buf[8];
	int16_t sample;

	// Wait for start of data header
	do {
		read_n_bytes (fd,&c,1);
		fprintf (stderr,"%02x .",c);
	} while (c != START_DATA_HEADER);

	//fprintf (stderr,"*** START_OF_DATA ***\n");

	// Read packet type
	read_n_bytes (fd,&c,1);
	fprintf (stderr,"c=%02x\n",c);

	switch (c) {
		case CMD_DATA_STREAMING:
			read_n_bytes (fd,&heart_rate,1);
			read_n_bytes (fd,&respiration,1);
			read_n_bytes (fd,&lead_off,1);

			fprintf (stderr,"heart_rate=%d\nrespiration=%d\nlead_off=%d\n",heart_rate,respiration,lead_off);

			for (i = 0; i < 14; i++) {
				read_n_bytes (fd,&sample,2);
				fprintf (stdout,"%d ", sample);
				read_n_bytes (fd,&sample,2);
				fprintf (stdout,"%d\n", sample);
			}
			break;

		case CMD_REG_READ:
			read_n_bytes (fd,buf,5);
			v = buf[1];
			fprintf (stdout,"%x\n",v);
			break;

		case CMD_QUERY_FIRMWARE_VERSION:
			read_n_bytes (fd,buf,2);
			fprintf (stdout,"%d.%d\n",buf[0],buf[1]);
			break;
		
		default:
			fprintf (stderr,"unknown packet type %x\n",c);
			do {
				read_n_bytes(fd,buf,1);
				display_hex(buf,1);
			} while (buf[0] != END_DATA_HEADER);
			fprintf (stderr, "\n");
	}

	return 0;
}

/**
 * Write a command to ADS1x9x EVM. Commands are:
 * CMD_REG_WRITE (0x91): register, value
 * CMD_REG_READ (0x92): register, 0x00
 * CMD_DATA_STREAMING (0x93): on/off, 0x00  (0x00 = off, 0x01 = on)
 *
 * @return Always 0.
 */

int ads1x9x_evm_write_cmd (int fd, int cmd, int param0, int param1) {

	cmd_buf[0] = START_DATA_HEADER;
	cmd_buf[1] = cmd;
	cmd_buf[2] = param0;
	cmd_buf[3] = param1;
	cmd_buf[4] = END_DATA_HEADER;
	cmd_buf[5] = END_DATA_HEADER;
	cmd_buf[6] = 0x0A;

	write (fd, &cmd_buf, 7);

	return 0;
}

int main( int argc, char **argv) {

	int speed = 9600;
	int stream_format = FORMAT_DECIMAL;

	char *device;
	char *command;
	char *param;

	// Setup signal handler. Catching SIGPIPE allows for exit when 
	// piping to Wireshark for live packet feed.
	//signal(SIGPIPE, signal_handler);
	struct sigaction act;
	memset(&act, 0, sizeof(act));
	act.sa_sigaction = signal_handler;
	act.sa_flags = SA_SIGINFO;
	sigaction(SIGPIPE, &act, NULL);


	// Parse command line arguments. See usage() for details.
	int c;
	while ((c = getopt(argc, argv, "b:c:d:f:hqs:t:v")) != -1) {
		switch(c) {
			case 'b':
				speed = atoi (optarg);
				break;
			case 'd':
				debug_level = atoi (optarg);
				break;

			case 'f':
				if (optarg[0] = 'b') {
					stream_format = FORMAT_BINARY;
				}
				break;
			

			case 'h':
				version();
				usage();
				exit(EXIT_SUCCESS);

			case 'q':
				quiet_mode = TRUE;
				break;

			case 'v':
				version();
				exit(EXIT_SUCCESS);
			case '?':	// case when a command line switch argument is missing
				/*
				if (optopt == 'c') {
					fprintf (stderr,"ERROR: 802.15.4 channel 11 to 26 must be specified with -c\n");
					exit(-1);
				}
				*/
				if (optopt == 'd') {
					fprintf (stderr,"ERROR: debug level 0 .. 9 must be specified with -d\n");
					exit(-1);
				}
				break;
		}
	}

	// One parameters are mandatory
	if (argc - optind < 1) {
		fprintf (stderr,"Error: missing command arguments. Use -h for help.");
		exit(EXIT_FAILURE);
	}

	device = argv[optind];
	command = argv[optind+1];

	if (debug_level > 0) {
		debug (1,"device=%s",device);
		debug (1,"command=%d",command);
	}

	if (debug_level > 0) {
		fprintf (stderr,"DEBUG: debug level %d\n",debug_level);
	}
	
	// Open device
	int fd = ads1x9x_evm_open(device,speed);
	if (fd < 1 ) {
		fprintf (stderr,"Error: unable to open device %s\n", device);
		return EXIT_FAILURE;
	}


	// Ignore anything aleady in the buffer
	tcflush (fd,TCIFLUSH);

	ads1x9x_evm_frame_t frame;


	if (strcmp("readreg",command)==0) {
		int reg = atoi(argv[optind+2]);
		ads1x9x_evm_write_cmd(fd,CMD_REG_READ,reg,0x00);
		ads1x9x_evm_read_frame (fd, &frame);
		fprintf (stdout, "%x\n", frame.data[1]);
	}

	if (strcmp("writereg",command)==0) {
		int reg = atoi(argv[optind+2]);
		int val = atoi(argv[optind+3]);
		ads1x9x_evm_write_cmd(fd,CMD_REG_WRITE,reg,val);
		ads1x9x_evm_read_response(fd);
	}


	if (strcmp("filter",command)==0) {
		int filterOpt = atoi(argv[optind+2]);
		// Not clear what the purpose of the first param is. FW code ignores
		// the filter command if not 0,2,3, but is otherwise not used.
		ads1x9x_evm_write_cmd(fd,CMD_FILTER_SELECT,0x03,filterOpt);
		ads1x9x_evm_read_frame (fd, &frame);
	}

	// Start continuous data streaming by issuing ADS1x9x Read Data Continuous (RDATAC) command.
	// Streamed data is DSP processed by EVM and is 16 bits per sample.

	if (strcmp("stream",command)==0) {
		int nframe = atoi(argv[optind+2]);

		// Turn on continuous data streaming. This works as a toggle command. Parameters
		// are ignored.
		ads1x9x_evm_write_cmd(fd,CMD_DATA_STREAMING,0x00,0x00);

		int i,j;
		uint8_t hr,resp,loff;
		int16_t sample;
		for (j = 0; j < nframe; j++) {
			ads1x9x_evm_read_frame (fd, &frame);
			hr = frame.data[0];
			resp = frame.data[1];
			loff = frame.data[2];
			
			for (i = 0; i < 14; i++) {
				// TODO: can we just cast sample rather than all this bit fiddling
				sample = frame.data[i*4 + 3]<<8 | frame.data[i*4 + 4];
				fprintf (stdout,"%d ", sample);
				sample = frame.data[i*4 + 5]<<8 | frame.data[i*4 + 6];
				fprintf (stdout,"%d ", sample);
				fprintf (stdout,"%d %d %d\n",hr,resp,loff);
			}
		}
		// Turn off continuous data streaming
		ads1x9x_evm_write_cmd(fd,CMD_DATA_STREAMING,0x00,0x00);
	}

	if (strcmp("firmware",command)==0) {
		ads1x9x_evm_write_cmd(fd,CMD_QUERY_FIRMWARE_VERSION,0x00,0x00);
		ads1x9x_evm_read_frame (fd, &frame);

		ads1x9x_evm_read_response(fd);
	}
	if (strcmp("restart",command)==0) {
		ads1x9x_evm_write_cmd(fd,CMD_RESTART,0x00,0x00);
		ads1x9x_evm_read_response(fd);
	}
	if (strcmp("acquire_data",command)==0) {
		int nsamples = atoi(argv[optind+2]);
		nsamples &= 0xffff;
		fprintf (stderr,"nsamples=%d\n",nsamples);
		//ads1x9x_evm_write_cmd(fd,CMD_ACQUIRE_DATA,nsamples>>8,nsamples&0xff);
		ads1x9x_evm_write_cmd(fd,CMD_ACQUIRE_DATA,nsamples&0xff,nsamples>>8);
		ads1x9x_evm_read_response(fd);
	}
	if (strcmp("packet_read",command)==0) {
		ads1x9x_evm_read_response(fd);
	}
	ads1x9x_evm_close(fd);

	debug (1, "Normal exit");
	return EXIT_SUCCESS; 
}
