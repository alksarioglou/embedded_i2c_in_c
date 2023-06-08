# Embedded Firmware Project for I2C in C

The program initially reads the value of the tilt register from the shield's sensor, and stores it in the payload 
field of a packet of an experimental protocol. The packet is then stored in the EEPROM at an address 
given in the details below.

Then, the program implements the use of 'acknowledge polling' during page write cycles and the addition of a 
1's complement checksum.
