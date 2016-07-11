
#ifndef OPENBCI_C_H
#define OPENBCI_C_H

#define _POSIX_SOURCE 1 //POSIX compliant source
#define _BSD_SOURCE

#include <stdlib.h>
#include <termios.h>                
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>                  
#include <sys/signal.h>
#include <sys/types.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <time.h>


#define BUFFERSIZE 2048
#define TRUE 1
#define FALSE 0
#define BAUDRATE B115200 //Baudrate for OpenBCI (115200bps)


#define ADS1299_VREF 4.5
#define ACCEL_SCALE_FAC (0.002 * pow(2,4))


/* Structs */
//Struct of a packet, to be filled with packet data.
struct packet{
    float output[12];
    int isComplete;
};


/* Function Definitions */

extern void signal_handler_IO(int status);

extern void set_port(char* input);

extern void open_port();

extern void setup_port();

extern void not_streaming();

extern void clear_buffer();

extern void shift_buffer_down();

extern void print_packet(struct packet P);

extern void printString();

extern int bufferHandler(unsigned char buf[], int isStreaming);

extern int close_port();

extern int send_to_board(char* message);

extern struct packet byte_parser(unsigned char buf[], int res);

extern struct packet streaming();

#endif
