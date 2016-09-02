/*

This program provides serial port communication with the OpenBCI Board.

*/

#include "openbci_c.h"

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
#include <setjmp.h>
#include <sys/utsname.h>

#define BUFFERSIZE 4056
#define TRUE 1
#define FALSE 0
#define BAUDRATE B115200 //Baudrate for OpenBCI (115200bps)


#define ADS1299_VREF 4.5
#define ACCEL_SCALE_FAC (0.002 / pow(2,4))

/*Device context*/
struct openbci_t {
  int fd;                                                // Serial port file descriptor
  char* port;
  int numBytesAdded;                                     //Used for determining if a packet was sent
  int lastIndex;                                         //Used to place bytes into the buffer
  int gain_setting;
  int previous_sample;                                   //stos SIGSEGV error
  struct sigaction saio;
  int isStreaming;
  
  unsigned char parseBuffer[BUFFERSIZE];                 //Array of Unsigned Chars, initilized with the null-character ('\0')
  
  int dollaBills;                                        //Used to determine if a string was sent (depreciated?)
};


/**
*     Function: obci_create
*     ---------------------
*     Allocates memory for an OpenBCI device/dongle context, and
*     initializes internal values.
*
*/
void obci_create(openbci_t** obci){
  (*obci) = malloc(sizeof(openbci_t));
  (*obci)->port = "/dev/ttyUSB0";
  (*obci)->numBytesAdded = 0;
  (*obci)->lastIndex = 0;
  (*obci)->gain_setting = 24;
  (*obci)->previous_sample = 0;
  (*obci)->isStreaming = FALSE;
  memset((*obci)->parseBuffer, '\0', sizeof((*obci)->parseBuffer));
  (*obci)->dollaBills = 0;
}

/**
*     Function: obci_destroy
*     ----------------------
*     Frees resources associated with an OpenBCI device context.
*     The context is invalid after this call.
*
*/
void obci_destroy(openbci_t* obci){
  free(obci);
}

/**
*     Function: set_port
*     --------------------
*     Sets the name of the serial port that the dongle is connected to.
*     E.g. 'COM1', '/dev/ttyUSB0', '/dev/ttyACM0'
*
*/
void set_port(openbci_t* obci, char* input){ 
  obci->port = input;
}

/**
*     Function: open_port
*     -------------------
*     Opens the port specified by set_port()
*     If the open initially fails, the function continues to try to open the port until success
*
*     TODO: How to handle serial connection errors. Does the continuously retrying ever work? 
*           Should an error or -1 be returned after a while?
*/
int open_port(openbci_t* obci){
  int flags = O_RDWR | O_NOCTTY;          
  obci->fd = open(obci->port, flags);
  return obci->fd;
}


/**
*     Function: find_port
*     -------------------
*     Probes the system for available ports and opens the first one it finds.
*
*     TODO: Detect OpenBCI
*     TODO: implement platforms other than Linux
*/
void find_port(openbci_t* obci){
  //get values from the system itself (particularly utsname.sysname)
  struct utsname unameData;
  uname(&unameData);
  int return_val = 0;
  char stringLiteral[12];

  if(strcmp(unameData.sysname, "Linux") == 0 || strcmp(unameData.sysname, "cygwin")) {
    //Linux
    int repeat = TRUE;

    while(repeat==TRUE){
      for(int i = 0; i < 34; i++){
        sprintf(stringLiteral, "/dev/ttyUSB%i",i);
        set_port(obci, stringLiteral);
        return_val = open_port(obci);
        if(return_val == -1)
          printf("\nError opening on port /dev/ttyUSB%i %s\n",i,strerror(errno));  
        else
          return;
      }
      sleep(3);    
     
   }

  }

  else if(strcmp(unameData.sysname, "ERROR") == 0) printf("Windows\n");
  else if(strcmp(unameData.sysname, "darwin") == 0) printf("Darwin\n");

}

/**
*     Function: setup_port
*     ---------------------
*     Establishes the serial port attributes, based on specifications of the OpenBCI Board 
*     communications and data format.
*     After establishing attributes, sends a 'v' to the board for a soft reset.
*
*/
void setup_port(openbci_t* obci){

  /* Serial port settings */
  struct termios serialportsettings;
  tcgetattr(obci->fd,&serialportsettings);

  cfsetispeed(&serialportsettings,B115200);                   // set the input baud rate
  cfsetospeed(&serialportsettings,B115200);                   // set the output baud rate 

  //Control Flags
  serialportsettings.c_cflag &= ~PARENB;                      // set the parity bit (none)
  serialportsettings.c_cflag &= ~CSTOPB;                      // # of stop bits = 1 (2 is default)
  serialportsettings.c_cflag &= ~CSIZE;                       // clears the mask
  serialportsettings.c_cflag |= CS8;                          // set the # of data bits = 8
  serialportsettings.c_cflag |= CREAD;                        // turn on the receiver of the serial port (CREAD)
  serialportsettings.c_cflag |= CLOCAL;                       // no modem
  serialportsettings.c_cflag |= HUPCL;                        // drop DTR (i.e. hangup) on close

  //Input Data Flags
  serialportsettings.c_iflag = IGNPAR;                        // ignore parity errors
  serialportsettings.c_iflag &= ~(IXOFF | IXON | IXANY);      // ignore 'XOFF' and 'XON' command bits

  //Output Flags
  serialportsettings.c_oflag = 0;

  //Local Flags
  serialportsettings.c_lflag = 0;

  /*Special characters*/

  serialportsettings.c_cc[VMIN]=1;                            // minimum of 1 byte per read
  serialportsettings.c_cc[VTIME]=0;                           // minimum of 0 seconds between reads

  fcntl(obci->fd, F_SETFL, O_NDELAY|O_NONBLOCK );             // asynchronous settings
  tcsetattr(obci->fd,TCSANOW,&serialportsettings);            // set the above attributes
  tcflush(obci->fd, TCIOFLUSH);                               // flush the serial port
  send_to_board(obci, 'v');                                   // reset the board
}

/**
*     Function: parse_strings
*     -----------------------
*     Parses string data from the board (board identification, registry information, etc)
*
*/
void parse_strings(openbci_t* obci){
  int res;
  unsigned char buf[1];
  int howLong = 0;
  int wasTripped = FALSE;  
  int bufferVal = 0;

  while(1){
    usleep(5);
    res = read(obci->fd,buf,1);
    
    if(res > 0){
      if(howLong < -1000 && wasTripped == FALSE){
          howLong = 0; 
          wasTripped = TRUE;
      }
        bufferVal = buffer_handler(obci,buf,obci->isStreaming);
    }
    else if(howLong < -1000 && wasTripped == TRUE){ bufferVal = 1; howLong = 0; wasTripped = FALSE;}
    else if(howLong >= -1000) howLong += res;
    else if(howLong < -1000 && wasTripped == FALSE) return;
 
    if(bufferVal == 1) { print_string(obci); return;}
    else if(bufferVal == 2) ; //char was sent... may be useful in the future
    else if(bufferVal == 3) {obci->isStreaming = TRUE; break;} // a packet was sent. parse it 
    bufferVal = 0;
  }
  

}

/** 
*     Function: streaming
*     ----------------------
*     Controls the streaming loop
*
*
*/
openbci_packet_t streaming(openbci_t* obci){
  int res;
  unsigned char buf[1];
  int howLong = 0;
  openbci_packet_t packet; //a packet so nice I named it twice :^)


  /* Streaming loop */
  /* NOTES: STOP==FALSE is always true right now... infinite loop */
  while (1) {

     usleep(1); //Sleep for 1 microsecond (helps CPU usage)
     res = read(obci->fd, buf,1);
     if(res > 0) {
       howLong = 0;
       buffer_handler(obci,buf,obci->isStreaming);

       if(obci->numBytesAdded >= 33){ 
         packet = byte_parser(obci,obci->parseBuffer,33);
         if(packet.isComplete == TRUE) return packet;
       }
     }

     else if (howLong < -1000){
       clear_buffer(obci);  
       obci->isStreaming = FALSE;
       howLong = 0;
       break;
     }
     else if (howLong >= -1000) howLong += res;
   }

  return packet;
}


/**
*     Function: close_port
*     -----------------------
*     Closes the serial port
*
*/
int close_port(openbci_t* obci){
    return close(obci->fd);
}

/**
*     Function: send_to_board
*     -------------------------
*     Sends bytes to board
*
*/
int send_to_board(openbci_t* obci, char message){
    return write(obci->fd,&message,1);
}


/** 
*    Function: buffer_handler
*    ------------------------
*    Places data from the serial buffer to the parseBuffer for more parsing 
*    (and to prevent data loss)
*   
*    Returns:
*       1 if a string was recently sent
*       2 if a char was sent (may be useful for malformed packet debugging). 
*         Includes newline characters and spaces
*       3 if a start byte was sent (0xA0)
*       0 for other data
*      -1 for errors
**/
int buffer_handler(openbci_t* obci, unsigned char buf[],int isStreaming){
    if(obci->lastIndex >= sizeof(obci->parseBuffer)){
        printf("\nBuffer overflow !\n");
        return -1;
    }
    if(obci->isStreaming == FALSE){
        obci->parseBuffer[obci->lastIndex] = buf[0];
        obci->lastIndex++;
        
        if(buf[0] == '$'){
          //there's a string coming in the future... need 3 though to print it.
          obci->dollaBills++;
        }
        else if(buf[0] != '$' && obci->dollaBills > 0) obci->dollaBills = 0; //Keeps them dolla bills in check :^)

        if(obci->dollaBills == 3){obci->dollaBills = 0; return 1;} //must have printed a string...
        else if(isalpha(buf[0]) || buf[0] == '\n' || buf[0] == ' ') return 2;
        else if(buf[0] == 0xA0) return 3;
        else return 0;
    }
    else if(obci->isStreaming == TRUE){
        obci->parseBuffer[obci->lastIndex] = buf[0];
        obci->lastIndex++;
        obci->numBytesAdded++;
    }
    return -1;
}


/**
*    Function: print_string
*    -----------------------
*    Print string messages received from the board.
* 
*    Returns:
*        0 if printing  
*       -1 if "Error: No strings to print while streaming"
*/
int print_string(openbci_t* obci){
  if (obci->isStreaming){
    perror("Error: No strings to print while streaming");
    return -1;
  }else{
    for(int i = 0; i < obci->lastIndex; i++){printf("%c",obci->parseBuffer[i]); obci->parseBuffer[i] = '\0';}
    printf("\n");
    obci->lastIndex = 0;
    return 0;
  }
}


/* Shifts the buffer down by 1 index, clears the last index*/
void shift_buffer_down(openbci_t* obci){

    obci->lastIndex--;
    obci->numBytesAdded--;

    for(int i = 0; i < obci->lastIndex; i++) obci->parseBuffer[i] = obci->parseBuffer[i + 1];
    obci->parseBuffer[obci->lastIndex] = '\0';
}

/* Clears the buffer */
void clear_buffer(openbci_t* obci){

    for(int i = 0; i < obci->lastIndex; i++) obci->parseBuffer[i] = '\0';
    obci->lastIndex = 0;
    obci->numBytesAdded = 0;
}

/* Prints the packet passed to it */
void print_packet(openbci_packet_t p){
    printf("\nSAMPLE NUMBER %g\n",p.output[0]);
    int acc_channel = 0;
    
    for(int i = 1; i <= 8; i++) printf("Channel Number %i : %g\n",i,p.output[i]);

    for(int i = 9; i <= 11; i++) printf("Acc Channel %i : %f\n",acc_channel++, p.output[i]);

}

/*
*    Function: stream_started
*    -------------
*    Return: TRUE if already started
*            FALSE if not streaming
*/
int stream_started(openbci_t* obci){
  return obci->isStreaming;
}

/*
*    Function: start_stream
*    -------------
*    Starts streaming data from the OpenBCI Board.
*
*    Return: 0 if called send_to_board
*           -1 if "Error: Already streaming"
*/
int start_stream(openbci_t* obci){
  if (obci->isStreaming == TRUE){
    perror("Error: Already streaming");
    return -1;
  }else{
    printf("Starting stream...");
    send_to_board(obci, 'b');
    obci->isStreaming = TRUE;
    return 0;
  }
}

/*
*    Function: stop_stream
*    -------------
*    Stop streaming data from the OpenBCI Board.
*
*    Return: 0 if called to send_to_board
*           -1 if "Error: Not current streaming"
*/
int stop_stream(openbci_t * obci){
  if (obci->isStreaming == FALSE){
    perror("Error: Not currently streaming");
    return -1;
  }else{
    send_to_board(obci, 's');
    obci->isStreaming = FALSE;
    return 0;
  }
}

/**
*    Function: byte_parser
*    ---------------------
*    Parses the incoming bytes during streaming
*
*/
openbci_packet_t byte_parser (openbci_t* obci, unsigned char buf[], int res){
  static int channel_number = 0;                              // channel number (0-7)
  static int acc_channel = 0;                                 // accelerometer channel (0-2)
  static int byte_count = 0;                                  // keeps track of channel bytes as we parse
  static int temp_val = 0;                                    // holds the value while converting channel values from 24 to 32 bit integers
  static float temp_float = 0.0;
  openbci_packet_t packet;           // buffer to hold the output of the parse (all -data- bytes of one sample)
  int parse_state = 0;                                        // state of the parse machine (0-5)
  int is_parsing = TRUE; 


 
  if(buf[0] != 0xA0){ shift_buffer_down(obci); is_parsing=FALSE; }
  else if(buf[0] == 0xA0) parse_state = 1;


  
  while(is_parsing == TRUE && obci->lastIndex > 1){
    switch(parse_state){

    case 1:
        shift_buffer_down(obci);
    
        int sample_num = obci->parseBuffer[0];
        if(sample_num - obci->previous_sample > 20  || 
           sample_num == obci->previous_sample ) { packet.isComplete = FALSE; return packet;}
        
        obci->previous_sample = sample_num;

        packet.output[0] = sample_num;
        parse_state++;
    
        break;

    case 2:
        shift_buffer_down(obci);
        temp_val |= (((unsigned int)obci->parseBuffer[0]) << (16 - (byte_count*8)));
        byte_count++;
        if(byte_count == 3){

            if((temp_val & 0x00800000) > 0){
                    temp_val |= 0xFF000000;
                }
                else temp_val &= 0x00FFFFFF;
            //convert from counts to uVolts
            temp_val = (ADS1299_VREF/obci->gain_setting/(pow(2,23) - 1) * 1000000.f) * temp_val; 
            packet.output[++channel_number] = temp_val;
            

            if(channel_number == 8){
                parse_state++;
                byte_count = 0;
                temp_val = 0;
                acc_channel = 0;
            }
            else{
                byte_count = 0;
                temp_val = 0;
            }


        }
        break;

    case 3:
        shift_buffer_down(obci);
        temp_val |= (((unsigned int)obci->parseBuffer[0]) << (8 - (byte_count*8)));
        byte_count++;

        if (byte_count==2) {
          if ((temp_val & 0x00008000) > 0) {
            temp_val |= 0xFFFF0000;
          } else {
            temp_val &= 0x0000FFFF;
          }
          temp_float = temp_val * ACCEL_SCALE_FAC;        //convert from counts to G

          packet.output[acc_channel++ + 9] = temp_float;

        if (acc_channel==3) {                       // all channels arrived !
          parse_state++;
          byte_count=0;
          channel_number=0;
          temp_val=0;
        }else { byte_count=0; temp_val=0; temp_float=0; }
          }
        break;

    case 4:
        shift_buffer_down(obci);
        if(obci->parseBuffer[0] == 0xC0){packet.isComplete = TRUE;return packet;}
    }




  }
  packet.isComplete = FALSE;
  return packet;
}
