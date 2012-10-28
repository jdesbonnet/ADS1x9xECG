/**
 * ads1292r_evm.c- a command line utility to communicate with the
 * TI ADS1292R EVM board running stock firmware.
 *
 * Author: Joe Desbonnet, jdesbonnet@gmail.com
 * 
 *
 * Version 0.1 (13 September 2012)
 * 
 * To compile:
 * gcc -o ads1292r_evm ads1292r_evm.c -lrt
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


#define APP_NAME "ads1292r_evm"
#define VERSION "0.1, 13 Sep 2012"

#define TRUE 1
#define FALSE 0

#define FORMAT_DECIMAL 1
#define FORMAT_BINARY 2

// Command definitions from ADS1x9x_USB_Communication.h
#define START_DATA_HEADER			0x02
#define WRITE_REG_COMMAND			0x91
#define READ_REG_COMMAND			0x92
#define DATA_STREAMING_COMMAND		0x93
#define DATA_STREAMING_PACKET		0x93
#define ACQUIRE_DATA_COMMAND		0x94
#define ACQUIRE_DATA_PACKET 		0x94
#define PROC_DATA_DOWNLOAD_COMMAND	0x95
#define DATA_DOWNLOAD_COMMAND		0x96
#define FIRMWARE_UPGRADE_COMMAND	0x97
#define START_RECORDING_COMMAND		0x98
#define FIRMWARE_VERSION_REQ		0x99
#define STATUS_INFO_REQ 			0x9A
#define FILTER_SELECT_COMMAND		0x9B
#define ERASE_MEMORY_COMMAND		0x9C
#define RESTART_COMMAND				0x9D


#define END_DATA_HEADER				0x03

// ADS1292R registers
// RegAddr RegName: Bit7 Bit6 .. Bit0 [value on reset]
// 0x00 ID: REV_ID7 REV_ID6 REV_ID5 1 0 0 REV_ID1 REV_ID0 [factory programmed]
// 0x01 CONFIG1: SINGLE-SHOT 0 0 0 0 DR2 DR1 DR0 [0x02 on reset]
// 0x02 CONFIG2: 1 PBD_LOFF_COMP PDB_REFBUF VREF_4V CLK_EN 0 INT_TEST TEST_FREQ [0x80 on reset]
// 0x03 LOFF: COMP_TH2 COMP_TH1 COMP_TH0 1 ILEAD_OFF1 ILEAD_OFF0 0 FLEAD_OFF [0x10 on reset]
// 0x04 CH1SET: PD1 GAIN1_2 GAIN1_1 GAIN1_0 MUX1_3 MUX1_2 MUX1_1 MUX1_0 [0x00]
// 0x05 CH1SET: PD2 GAIN2_2 GAIN2_1 GAIN2_0 MUX2_3 MUX2_2 MUX2_1 MUX2_0 [0x00]

#define REG_ID 0x00


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
int ads1292r_evm_open(char *deviceName, int bps) {

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
void ads1292r_evm_close(int fd) {
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
	fprintf (stderr,"Usage: ads1292r_evm [-q] [-v] [-h] [-d level] device\n");

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
 *
 * @return The entire frame length (excluding cksum) if successful, -1 on error.
 */
int ads1292r_evm_read_response (int fd) {
	
	int i;
	uint8_t c,v,heart_rate,respiration,lead_off;
	uint8_t buf[8];
	int16_t sample;

	// Wait for start of data header
	do {
		read_n_bytes (fd,&c,1);
		//fprintf (stderr,"%02x .",c);
	} while (c != START_DATA_HEADER);

	//fprintf (stderr,"*** START_OF_DATA ***\n");

	// Read packet type
	read_n_bytes (fd,&c,1);
	//fprintf (stderr,"c=%02x\n",c);
	switch (c) {
		case DATA_STREAMING_PACKET:
		read_n_bytes (fd,&heart_rate,1);
		read_n_bytes (fd,&respiration,1);
		read_n_bytes (fd,&lead_off,1);

		//fprintf (stderr,"heart_rate=%d\nrespiration=%d\nlead_off=%d\n",heart_rate,respiration,lead_off);

		for (i = 0; i < 14; i++) {
			read_n_bytes (fd,&sample,2);
			fprintf (stdout,"%d ", sample);
			read_n_bytes (fd,&sample,2);
			fprintf (stdout,"%d\n", sample);
		}
		break;

		case READ_REG_COMMAND:
		read_n_bytes (fd,buf,5);
		v = buf[1];
		fprintf (stdout,"regval=%x\n",v);
		break;
		
		default:
		fprintf (stderr,"unknown packet type\n");
		read_n_bytes (fd,&sample,2);
		fprintf (stdout,"%0x\n",sample);
	}

	return 0;
}

/**
 * Write a command. Commands are:
 * WRITE_REG_COMMAND (0x91): register, value
 * READ_REG_COMMAND (0x92): register, 0x00
 * DATA_STREAMING_PACKET (0x93): on/off, 0x00  (0x00 = off, 0x01 = on)
 *
 * @return Always 0.
 */

int ads1292r_evm_write_cmd (int fd, int cmd, int param0, int param1) {

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
	int fd = ads1292r_evm_open(device,speed);
	if (fd < 1 ) {
		fprintf (stderr,"Error: unable to open device %s\n", device);
		return EXIT_FAILURE;
	}


	// Ignore anything aleady in the buffer
	tcflush (fd,TCIFLUSH);

	if (strcmp("readreg",command)==0) {
		int reg = atoi(argv[optind+2]);
		ads1292r_evm_write_cmd(fd,READ_REG_COMMAND,reg,0x00);
		ads1292r_evm_read_response(fd);
	}

	if (strcmp("writereg",command)==0) {
		int reg = atoi(argv[optind+2]);
		int val = atoi(argv[optind+3]);
		ads1292r_evm_write_cmd(fd,READ_REG_COMMAND,reg,val);
		ads1292r_evm_read_response(fd);
	}

	if (strcmp("stream",command)==0) {
		ads1292r_evm_write_cmd(fd,DATA_STREAMING_COMMAND,0x01,0x00);
		while (1) {
			ads1292r_evm_read_response (fd);
		}
	}

	ads1292r_evm_close(fd);

	debug (1, "Normal exit");
	return EXIT_SUCCESS; 
}
