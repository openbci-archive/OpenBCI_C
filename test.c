#include "openbci_c.h"


int main(){
  struct packet local_packet;
  int return_val = 0;
  char * port = "/dev/ttyUSB0";

  find_port();
  set_port(port);


  setup_port();

  parse_strings();
  sleep(5);
  start_stream();

  while(1){
    local_packet = streaming();
    print_packet(local_packet);
  }

  return return_val;
}

