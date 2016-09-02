#include "openbci_c.h"

#include <unistd.h>


int main(){
  openbci_packet_t local_packet;
  openbci_t * obci;
  int return_val = 0;

  obci_create(&obci);
  set_port(obci, "/dev/ttyUSB0");
  find_port(obci);


  setup_port(obci);

  while(1){
    if (stream_started(obci)){

      local_packet = streaming(obci);
      print_packet(local_packet);

    }else{ // stream_started() == FALSE

      parse_strings(obci);
      sleep(5);
      start_stream(obci);

    }
  }

  obci_destroy(obci);

  return return_val;
}

