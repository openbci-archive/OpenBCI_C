#include "openbci_c.h"

int main(){
	find_port();
	int fd = open_port();
	setup_port();

	while(1) {
		unsigned char buf[1];
		int res;
		res = read(fd, buf, 1);
		write(STDOUT_FILENO, buf, res);
	}
}
