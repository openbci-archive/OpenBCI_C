#include "openbci_c.h"

#include <unistd.h>

int main(){
  openbci_t* obci;
  obci_create(&obci, find_port());
  int fd = obci_fd(obci);

  while(1) {
    unsigned char buf[1];
    int res;
    res = read(fd, buf, 1);
    write(STDOUT_FILENO, buf, res);
  }

  obci_destroy(obci);
  return 0;
}
