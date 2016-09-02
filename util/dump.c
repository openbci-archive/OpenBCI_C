#include "openbci_c.h"

#include <unistd.h>

int main(){
  openbci_t* obci;
  obci_create(&obci);
  
  find_port(obci);
  close_port(obci);
  int fd = open_port(obci);
  setup_port(obci);

  while(1) {
    unsigned char buf[1];
    int res;
    res = read(fd, buf, 1);
    write(STDOUT_FILENO, buf, res);
  }
}
