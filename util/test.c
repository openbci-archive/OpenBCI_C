#include "openbci_c.h"

#include <unistd.h>


int main(){
  openbci_packet_t local_packet;
  openbci_t * obci;
  int return_val = 0;

  char const* port = obci_find_port();
  if (!port)
    port = "/dev/ttyUSB0";
  obci_create(&obci, port);
  obci_reset(obci);

  while(1){
    if (obci_stream_started(obci)){

      local_packet = obci_streaming(obci);
      obci_print_packet(local_packet);

    }else{ // obci_stream_started() == FALSE

      obci_parse_strings(obci);
      sleep(5);
      obci_start_stream(obci);

    }
  }

  obci_destroy(obci);

  return return_val;
}

