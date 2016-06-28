/*

This program provides serial port communication with the OpenBCI Board.

*/
#define _POSIX_SOURCE 1       // POSIX compliant source

#include <stdio.h>                  
#include <termios.h>                
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>                  
#include <sys/signal.h>
#include <sys/types.h>
#include <math.h>
#include <string.h>

#define BAUDRATE B115200            // define baudrate (115200bps)
#define PORT "/dev/ttyUSB0"         // define port
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE; 

/*Function declarations*/
void signal_handler_IO (int status);
void byte_parser (unsigned int* buf, int res);
void open_port();
void setup_port();
void streaming();
int close_port();
int send_to_board(char* message);

/*Global variables*/
int fd;                                                        // Serial port file descriptor
char* port;
int wait_flag=FALSE;                                           // Signal handler wait flag
int* buf[1024];                                                // Buffer to holder reads
struct termios serialportsettings;                             // Serial port settings
struct sigaction saio;                                         // Signal handler            
    
void main(){
  port = "/dev/ttyUSB0";
  open_port();
  setup_port();
  streaming();
  // return;
}

void open_port(){
  int flags = O_RDWR | O_NOCTTY;          
  fd = open(port, flags);
  if (-1 == fd){
    /* Some sort of error handling */
    printf("ERROR! In opening ttyUSB0\n");  
  }
}

void setup_port(){
  tcgetattr(fd,&serialportsettings);
  /* Serial port settings */
  cfsetispeed(&serialportsettings,B115200);                   // set the input baud rate
  cfsetospeed(&serialportsettings,B115200);                   // set the output baud rate 
  //Control Flags
  serialportsettings.c_cflag &= ~PARENB;                      // set the parity bit (none)
  serialportsettings.c_cflag &= ~CSTOPB;                      // # of stop bits = 1 (2 is default)
  serialportsettings.c_cflag &= ~CSIZE;                       // clears the mask
  serialportsettings.c_cflag |= CS8;                          // set the # of data bits = 8
  // serialportsettings.c_cflag &= ~CRTSCTS;                     // no hardware based flow control (RTS/CTS)
  serialportsettings.c_cflag |= CREAD;                        // turn on the receiver of the serial port (CREAD)
  serialportsettings.c_cflag |= CLOCAL;                       // no modem
  serialportsettings.c_cflag |= HUPCL;                        // ?? drop DTR (i.e. hangup) on close
  //Input Data Flags
  serialportsettings.c_iflag = IGNPAR;                        // *NEW* ignore parity errors
  serialportsettings.c_iflag &= ~(IXOFF | IXON | IXANY);       //ignore 'XOFF' and 'XON' command bits [fixes a bug where the parser skips '0x11' and '0x13']
  //Output Flags
  serialportsettings.c_oflag = 0;
  //Local Flags
  serialportsettings.c_lflag = 0;
  /*Special characters*/

  serialportsettings.c_cc[VMIN]=1;                            // minimum of 1 byte per read
  serialportsettings.c_cc[VTIME]=0;                           // minimum of 0 seconds between reads

  fcntl(fd, F_SETFL, O_NDELAY|O_ASYNC|O_NONBLOCK );                      // asynchronous settings
  tcsetattr(fd,TCSANOW,&serialportsettings);                  // set the above attributes
  saio.sa_handler = signal_handler_IO;                        // signal handling
  saio.sa_flags = 0;                                          // signal handling
  saio.sa_restorer = NULL;                                    // signal handling
  sigaction(SIGIO,&saio,NULL);                                // signal handling
  tcflush(fd, TCIOFLUSH);                                     // flush the serial port
  send_to_board("v");                                         // reset the board and receive+print board information by sending a "v" command
}

void streaming(){
  int res;
  /* Streaming loop */
  while (STOP==FALSE) {
    // signal
    if (wait_flag==FALSE) { 
      res = read(fd,&buf,1);                            // read 33 bytes from serial and place at buf
      printf("%d",res);
      byte_parser(*buf,res);                             // send the read to byte_parser()
      // memset(buf,0,res);
      wait_flag = TRUE;                                 // wait for new input //
    }
  }


    close_port();
}

/* CLOSE SERIAL PORT */
int close_port(){
    return close(fd);
}
/* Send messages to the board */
/* Return: bytes written */
int send_to_board(char* message){
    return write(fd,message,1);                                // possibly make the size dynamic?
}

/*Signalling*/
void signal_handler_IO (int status){
    wait_flag = FALSE;
}

/* Byte Parser */
void byte_parser (unsigned int* buf, int res){
  static unsigned char framenumber = -1;                      // framenumber = sample number from board (0-255)
  static int channel_number = 0;                              // channel number (0-7)
  static int acc_channel = 0;                                 // accelerometer channel (0-2)
  static int byte_count = 0;                                  // keeps track of channel bytes as we parse
  static int temp_val = 0;                                    // holds the value while converting channel values from 24 to 32 bit integers
  static float output[11];                                    // buffer to hold the output of the parse (all -data- bytes of one sample)
  int parse_state = 0;                                        // state of the parse machine (0-5)
  printf("######### NEW PACKET ##############\n");
  for (int i=0; i<res;i++){                                   // iterate over the contents of a packet
    printf("%d |", i);                                        // print byte number (0-33)
    printf("PARSE STATE %d | ", parse_state);                 // print current parse state
    printf("BYTE %x\n",buf[i]);                               // print value of byte

    /* Parser State Machine */
    switch (parse_state) {
      case 0:                                             // STATE 0: find end+beginning byte
        if (buf[i] == 0xC0){                            // if finds end byte first, look for beginning byte next
          parse_state++;                              
        }
        else if (buf[i] == 0xA0){                       // if find beginning byte first, proceed to parsing sample number (state 2)
          parse_state = 2;                                                    
        }
        break;
      case 1:                                             // STATE 1: Look for header (in case C0 found first)
        if (buf[i] == 0xA0){
          parse_state++;
        }else{
          parse_state = 0;
        }
        break;
      case 2:                                             // Check framenumber
        if (((buf[i]-framenumber)!=1) && (buf[i]==0)){  
          /* Do something like this to check for missing
                  packets. Keep track of missing packets. */
          printf("MISSING PACKET \n");
        }
        framenumber++;
        parse_state++;
        break;
      case 3:                                             // get ADS channel values **CHANNEL DATA**
        temp_val |= (((unsigned int)buf[i]) << (16 - (byte_count*8))); //convert to MSB
        byte_count++;   
        if (byte_count==3){                             // if 3 bytes passed, 24 bit to 32 bit conversion
            printf("CHANNEL NO. %d\n", channel_number + 1);
            if ((temp_val & 0x00800000) > 0) {
                temp_val |= 0xFF000000;
            }else{
                temp_val &= 0x00FFFFFF;
            }
            // temp_val = (4.5 / 24 / float((pow(2, 23) - 1)) * 1000000.f) * temp_val; // convert from count to bytes
            output[channel_number] = temp_val;          // place value into data output buffer
            channel_number++;
            if (channel_number==8){                     // check to see if 8 channels have already been parsed
                parse_state++;
                byte_count = 0;
                temp_val = 0;
                acc_channel = 0;
            }else{
                byte_count = 0;
                temp_val = 0;
            }
        }
        break;
      case 4:                                             // get LIS3DH channel values 2 bytes times 3 axes **ACCELEROMETER**
        temp_val |= (((unsigned int)buf[i]) << (8 - (byte_count*8)));
        byte_count++;
        if (byte_count==2) {
          if ((temp_val & 0x00008000) > 0) {
            temp_val |= 0xFFFF0000;
          } else {
            temp_val &= 0x0000FFFF;
          }  
          // printf("channel no %d\n", channel_number);
          printf("acc channel %d\n", acc_channel);
          output[acc_channel + 8]=temp_val;           // output onto buffer
          acc_channel++;
        if (acc_channel==3) {                       // all channels arrived !
          parse_state++;
          byte_count=0;
          channel_number=0;
          temp_val=0;
        }else { byte_count=0; temp_val=0; }
          }
        break;

      case 5:                                             // look for end byte
        if (buf[i] == 0xC0){
          parse_state = 0;
        }
        else{
         // something about synching here
          parse_state= 0;                             // resync
        }
        break;

      default: parse_state=0;     
    }
  }
  return;
}
