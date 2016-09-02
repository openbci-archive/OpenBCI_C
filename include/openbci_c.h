
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
  OBCI_SYSERROR     // system call failure, details in errno
};

/* Function Definitions */

extern int obci_create(openbci_t** obci, char const * port);

extern void obci_destroy(openbci_t* obci);

extern void obci_reset(openbci_t* obci);

extern void parse_strings(openbci_t* obci);

extern void clear_buffer(openbci_t* obci);

extern void shift_buffer_down(openbci_t* obci);

extern void print_packet(openbci_packet_t P);

extern int print_string(openbci_t* obci);

extern char const* find_port();

extern int obci_fd(openbci_t* obci);

extern int stream_started(openbci_t* obci);

extern int start_stream(openbci_t* obci);

extern int stop_stream(openbci_t* obci);

extern int buffer_handler(openbci_t* obci, unsigned char buf[], int isStreaming);

extern int send_to_board(openbci_t* obci, char message);

extern openbci_packet_t byte_parser(openbci_t* obci, unsigned char buf[], int res);

extern openbci_packet_t streaming(openbci_t* obci);


#endif
