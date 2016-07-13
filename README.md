# OpenBCI_C
C driver for OpenBCI boards.  Currently only prints out information.

In order to communicate with the board, you currently must `echo` directly to the board. For example...<br/>
`echo -n 'b' > /dev/ttyUSB0` to start streaming <br/>
`echo -n 's' > /dev/ttyUSB0` to stop streaming <br/>

Any other character can be sent, however only certain characters will do things to the board. Check out http://docs.openbci.com/software/01-OpenBCI_SDK for more info.

##To-Do
- Fix Address Boundary Error
- Error handling
- Implement tests
- Implementations with: Python, Labstreaminglayer, Matlab, Java/Processing

##Known Bugs
- SIGSEGV (Address Boundary Error) occasionally when streaming is halted
