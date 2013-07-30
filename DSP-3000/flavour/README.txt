This script allows the DSP-3000 USB interface cable to be "flavoured" with a specific product name.
Doing so allows the node to find the device in the /dev/clearpath folder.  

The USB interface must be an FTDI-brand USB to RS232 cable; eg. the USB-RS232-5.0

To use it, plug in *only* the DSP-3000 cable, then run the script.  It should report that new contents
have been written to the eeprom.  Unplug and replug the cable, and it should appear as /dev/clearpath/fog
