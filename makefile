main:
	gcc -c -o openbci_c.o openbci_c.c -std=gnu99

test:
	gcc -o test test.c -std=gnu99

clean:
	rm -r *.o *.so 

python:
	gcc -Wall -fPIC -c openbci_c.c -std=gnu99
	gcc -shared -Wl,-soname,libopenbci.so.1 -o openbci.so openbci_c.o 
