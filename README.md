parled-dmx-5ch
==============

Replacement Firmware for Par-Led-56 wich enable 5 channels (R,G,B, Global Dimmer, Strobe). The Firmware will read the dip switch dmx address value at the begining of the runtime.


Necessary tool needed
=====================
- build-essential package
- sdcc package
- PIC programmer (tested with Pickit II)

How to build
============
Simply run go.sh

Or use : make clean && make All 
Then upload with : make write


