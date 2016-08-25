#include "openbci_c.h"


int main(){
  struct packet local_packet;
  int return_val = 0;

  set_port("/dev/ttyUSB0");
  find_port();


  setup_port();

  while(1){
    if (stream_started()){

      local_packet = streaming();
      print_packet(local_packet);

    }else{ // stream_started() == FALSE

      parse_strings();
      sleep(5);
      start_stream();     

    }
  }

  return return_val;
}

