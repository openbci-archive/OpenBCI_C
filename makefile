main:
	gcc -o openbci_c openbci_c.h openbci_c.c -std=gnu99

test:
	gcc -o test test.c openbci_c.h -std=gnu99

clean:
	rm -r openbci_c test



