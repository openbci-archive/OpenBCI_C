#include "openbci_c.h"
#include "openbci_c.c"


void main(){
  struct packet local_packet;
  int return_val = 0;
  port = "/dev/ttyUSB0";

  find_port();
  //set_port(port);


  setup_port();

  while(1){
    switch(isStreaming){
      case FALSE:
          not_streaming();
          break;

      case TRUE:
          local_packet = streaming();
          print_packet(local_packet);
          break;
    }
  }

}

