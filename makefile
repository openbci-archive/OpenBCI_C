CFLAGS=-Wall -fPIC -std=gnu99


all: libopenbci.a test python

python: libopenbci.so

test: test.o libopenbci.a

libopenbci.a: openbci_c.o
	ar ru $@ $^
	ranlib $@

clean:
	-rm -r *.o *.a *.so test

libopenbci.so: libopenbci.a
	gcc -shared -Wl,-soname,libopenbci.so.1 -o $@ $^
