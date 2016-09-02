# OpenBCI_C

C driver for OpenBCI boards.  A work in progress.

## Community Built Library

This library is not maintained by OpenBCI, but everyone is welcome to work on it in any capacity. Just send a pull request and we'll merge it. Try not to break anything! :)


## Building

```
./autogen.sh
./configure
make
```

## Running

`./test`

In order to communicate with the board, you currently must `echo` directly to the board. For example...<br/>
`echo -n 'b' > /dev/ttyUSB0` to start streaming <br/>
`echo -n 's' > /dev/ttyUSB0` to stop streaming <br/>

Any other character can be sent, however only certain characters will do things to the board. Check out http://docs.openbci.com/software/01-OpenBCI_SDK for more info.

##To-Do
- [ ] Redefine the packet structure to be more user friendly, including string data and names for accelerometer data
- [ ] Clean up streaming code to be simpler and more flexible
- [ ] Provide all functions of board
  - [ ] Enumerate functions here
- [x] Fix Address Boundary Error
- [ ] Error handling (what is meant here?)
- [ ] Implement tests
- [ ] Implementations with: Python, Labstreaminglayer, Matlab, Java/Processing

##Known Bugs
- [ ] Stock board does not reset properly on start and may occasionally begin streaming in a corrupt manner.  Library does not detect this.
