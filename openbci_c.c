/*

This program provides serial port communication with the OpenBCI Board.

*/

#include "openbci_c.h"

volatile int STOP=FALSE;

/*Global variables*/
int fd;                                                        // Serial port file descriptor
char* port = "/dev/ttyUSB0";
int wait_flag=FALSE;                                           // Signal handler wait flag
int numBytesAdded = 0;        //Used for determining if a packet was sent
int lastIndex = 0;            //Used to place bytes into the buffer
int gain_setting = 24;
int previous_sample = 0; //stos SIGSEGV error
struct sigaction saio;
struct termios serialportsettings;
int isStreaming = FALSE;

unsigned char parseBuffer[BUFFERSIZE] = {'\0'}; //Array of Unsigned Chars, initilized with the null-character ('\0')

int dollaBills = 0;           //Used to determine if a string was sent (depreciated?)

// Sets the port to a passed string
void set_port(char* input){
  port = input;
}

// Opens the port stated in set_port
void open_port(){
  int flags = O_RDWR | O_NOCTTY;          
  fd = open(port, flags);
  
  while(fd == -1){
    printf("ERROR! In opening ttyUSB0! Trying again...\n");  
    sleep(3);
    fd = open(port, flags);
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
  send_to_board("v");          // reset the board and receive+print board information by sending a "v" command

}

// Prints strings only
void not_streaming(){
  int res;
  unsigned char buf[1];
  int howLong = 0;
  int wasTripped = FALSE;  
  int bufferVal = 0;

  while(STOP==FALSE){
    usleep(5);
    res = read(fd,buf,1);
    
    if(res > 0){
      if(howLong < -10000000 && wasTripped == FALSE){
          howLong = 0; 
          wasTripped = TRUE;
      }
        bufferVal = bufferHandler(buf,isStreaming);
    }
    else if(howLong < -10000000 && wasTripped == TRUE){ bufferVal = 1; howLong = 0; wasTripped = FALSE;}
    else if(howLong >= -10000000) howLong += res;
      
    if(bufferVal == 1) printString();
    else if(bufferVal == 2) ; //char was sent... may be useful in the future
    else if(bufferVal == 3) {isStreaming = TRUE; break;} // a packet was sent. parse it 
    bufferVal = 0;
  }
  

}

// The main streaming function
struct packet streaming(){
  int res;
  unsigned char buf[1];
  int howLong = 0;
  int bufferVal = 0; //the value returned by bufferHandler
  int wasTripped = FALSE;
  char input;
  struct packet packet; //a packet so nice I named it twice :^)


  /* Streaming loop */
  /* NOTES: STOP==FALSE is always true right now... infinite loop */
  while (STOP==FALSE) {

     usleep(1); //Sleep for 1 microsecond (helps CPU usage)
     res = read(fd, buf,1);
     if(res > 0) {
       howLong = 0;
       bufferHandler(buf,isStreaming);

       if(numBytesAdded >= 33){ 
         packet = byte_parser(parseBuffer,33);
         if(packet.isComplete == TRUE) return packet;
       }
     }

     else if (howLong < -1000){
       clear_buffer();  
       isStreaming = FALSE;
       howLong = 0;
       break;
     }
     else if (howLong >= -1000) howLong += res;
   }


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

/** BUFFER HANDLER

    Places data from the serial buffer to the parseBuffer for more parsing (and to prevent data loss)

    returns...
        - 1 if a string was recently sent
        - 2 if a char was sent (may be useful for malformed packet debugging). Includes newline characters and spaces
        - 3 if a start byte was sent (0xA0)
        - 0 for other data
**/


int bufferHandler(unsigned char buf[],int isStreaming){

    if(isStreaming == FALSE){
        parseBuffer[lastIndex] = buf[0];
        lastIndex++;
        
        if(buf[0] == '$'){
          //there's a string coming in the future... need 3 though to print it.
          dollaBills++;
        }
        else if(buf[0] != '$' && dollaBills > 0) dollaBills = 0; //Keeps them dolla bills in check :^)

        if(dollaBills == 3){dollaBills = 0; return 1;} //must have printed a string...
        else if(isalpha(buf[0]) || buf[0] == '\n' || buf[0] == ' ') return 2;
        else if(buf[0] == 0xA0) return 3;
        else return 0;
    }
    else if(isStreaming == TRUE){
        parseBuffer[lastIndex] = buf[0];
        lastIndex++;
        numBytesAdded++;
    }

}


/* Prints strings and removes the chars from parseBuffer */

void printString(){

    for(int i = 0; i <= lastIndex; i++){printf("%c",parseBuffer[i]); parseBuffer[i] = '\0';}
    printf("\n");
    lastIndex = 0;
}


/* Shifts the buffer down by 1 index, clears the last index*/
void shift_buffer_down(){

    for(int i = 0; i < lastIndex; i++) parseBuffer[i] = parseBuffer[i + 1];
    parseBuffer[lastIndex] = '\0';  
    lastIndex--;
    numBytesAdded--;
    
}

/* Clears the buffer */
void clear_buffer(){

    for(int i = 0; i <= lastIndex; i++) parseBuffer[i] = '\0';
    lastIndex = 0;
    numBytesAdded = 0;
}

/* Prints the packet passed to it */
void print_packet(struct packet p){
    printf("\nSAMPLE NUMBER %g\n",p.output[0]);
    int acc_channel = 0;
    
    for(int i = 1; i <= 8; i++) printf("Channel Number %i : %g\n",i,p.output[i]);

    for(int i = 9; i <= 11; i++) printf("Acc Channel %i : %f\n",acc_channel++, p.output[i]);

}

/* Byte Parser */
struct packet byte_parser (unsigned char buf[], int res){
  static unsigned char framenumber = -1;                      // framenumber = sample number from board (0-255)
  static int channel_number = 0;                              // channel number (0-7)
  static int acc_channel = 0;                                 // accelerometer channel (0-2)
  static int byte_count = 0;                                  // keeps track of channel bytes as we parse
  static int temp_val = 0;                                    // holds the value while converting channel values from 24 to 32 bit integers
  static float temp_float = 0.0;
  struct packet packet;           // buffer to hold the output of the parse (all -data- bytes of one sample)
  int parse_state = 0;                                        // state of the parse machine (0-5)
  int is_parsing = TRUE; 


 
  if(buf[0] != 0xA0){ shift_buffer_down(); is_parsing=FALSE; }
  else if(buf[0] == 0xA0) parse_state = 1;


  
  while(is_parsing == TRUE){
    switch(parse_state){

    case 1:
        shift_buffer_down();
    
        int sample_num = parseBuffer[0];
        if(sample_num - previous_sample > 20  || 
           sample_num == previous_sample ) { packet.isComplete = FALSE; return packet;}
        
        previous_sample = sample_num;

        packet.output[0] = sample_num;
        parse_state++;
    
        break;

    case 2:
        shift_buffer_down();
        temp_val |= (((unsigned int)parseBuffer[0]) << (16 - (byte_count*8)));
        byte_count++;
        if(byte_count == 3){

            if((temp_val & 0x00800000) > 0){
                    temp_val |= 0xFF000000;
                }
                else temp_val &= 0x00FFFFFF;
            //convert from counts to uVolts
            temp_val = (ADS1299_VREF/gain_setting/(pow(2,23) - 1) * 1000000.f) * temp_val; 
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
        shift_buffer_down();
        temp_val |= (((unsigned int)parseBuffer[0]) << (8 - (byte_count*8)));
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
        shift_buffer_down();
        if(parseBuffer[0] == 0xC0){packet.isComplete = TRUE;return packet;}
    }




  }
  packet.isComplete = FALSE;
  return packet;
}
