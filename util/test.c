#include "openbci_c.h"

#include <unistd.h>


int main(){
  openbci_packet_t local_packet;
  openbci_t * obci;
  int return_val = 0;

  char const* port = find_port();
  if (!port)
    port = "/dev/ttyUSB0";
  obci_create(&obci, port);
  obci_reset(obci);

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

