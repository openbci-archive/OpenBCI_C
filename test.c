#include "openbci_c.h"
#include "openbci_c.c"

void main(){
  struct packet local_packet;

  port = "/dev/ttyUSB0";
  set_port(port);
  open_port();
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


