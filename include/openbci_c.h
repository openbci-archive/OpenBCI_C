
#ifndef OPENBCI_C_H
#define OPENBCI_C_H

/* Structs */
//Struct of a packet, to be filled with packet data.
typedef struct{
  float output[12];
  int isComplete;
}openbci_packet_t;

//OpenBCI device context
typedef struct openbci_t openbci_t;

/* Enums */

enum {
  OBCI_SUCCESS = 0, // success
  OBCI_SYSERROR,    // system call failure, details in errno
  OBCI_FUTILE       // the requested action did nothing
};

/* Function Definitions */

/*
*     Allocates memory for an OpenBCI device/dongle context, initializes
*     internal values, and establishes the serial port attributes, based
*     on specifications of the OpenBCI Board communications and data format.
*     Opens the port specified.  Returns OBCI_SYSERROR if the open fails.
*     Provide the name of the serial port that the dongle is connected to.
*     E.g. 'COM1', '/dev/ttyUSB0', '/dev/ttyACM0'
*/
extern int obci_create(openbci_t** obci, char const * port);


/*
*     Frees resources associated with an OpenBCI device context and
*     closes the serial port.
*     The context is invalid after this call.
*/
extern void obci_destroy(openbci_t* obci);


/*
*     Sends a 'v' to the board for a soft reset.
*/
extern void obci_reset(openbci_t* obci);


/*
*     Sends an arbitrary command to the board.
*/
extern int obci_send_to_board(openbci_t* obci, char message);


/*
*    Return: 1 if data is streaming
*            0 if not streaming
*/
extern int obci_stream_started(openbci_t* obci);


/*
*    Starts streaming data from the OpenBCI Board.
*
*    Return: OBCI_SUCCESS if command sent
*            OBCI_FUTILE if "Warning: Already streaming"
*/
extern int obci_start_stream(openbci_t* obci);


/*
*    Stops streaming data from the OpenBCI Board.
*
*    Return: OBCI_SUCCESS if command sent
*            OBCI_FUTILE if "Warning: Not current streaming"
*/
extern int obci_stop_stream(openbci_t* obci);


/*
*     Returns the open file descriptor for the port associated with a
*     device context.  For debugging.
*/
extern int obci_fd(openbci_t* obci);


/*
*     Probes the system for available ports and returns the first one
*     it finds.  The string is stored statically.
*     This function is incomplete and only works on linux.
*/
extern char const* obci_find_port();


/*
*     Prints on stdout a human-readable representation of the passed
*     packet.
*/
extern void obci_print_packet(openbci_packet_t P);


extern void obci_parse_strings(openbci_t* obci);

extern int obci_print_string(openbci_t* obci);

extern openbci_packet_t obci_streaming(openbci_t* obci);


#endif
