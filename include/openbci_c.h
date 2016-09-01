
#ifndef OPENBCI_C_H
#define OPENBCI_C_H

/* Structs */
//Struct of a packet, to be filled with packet data.
struct packet{
    float output[12];
    int isComplete;
};


/* Function Definitions */

extern void set_port(char* input);

extern void setup_port();

extern void parse_strings();

extern void clear_buffer();

extern void shift_buffer_down();

extern void print_packet(struct packet P);

extern int print_string();

extern void find_port();

extern int stream_started();

extern int start_stream();

extern int stop_stream();

extern int buffer_handler(unsigned char buf[], int isStreaming);

extern int close_port();

extern int send_to_board(char* message);

extern int open_port();

extern struct packet byte_parser(unsigned char buf[], int res);

extern struct packet streaming();


#endif
