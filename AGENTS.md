SDI-TRANS-I2C.

This code allows an I2C port control an SDI-12 bus.  The existing code has a serial port via USB.  The current code works.

We want to modify the code to add I2C slave capability using the SDA/SCL lines that are defined in the SysDep.h file.  

The specification for SDI-12 protocol is here -> https://www.sdi-12.org/current_specification/SDI-12_version-1_4-Jan-10-2019.pdf

The Kicad files for the board are here -> /home/metin/work/Vegetronix/SDI-TRANS/SDI-TRANS-I2C-Board

If possible create a reusable I2C Slave library.  Use the libraries that I've already created.  Do not modify them. If they need to be modified give me instructions.  

Do not remove the existing COM/USB port capability. 

The timing on SDI-12 is critical, so minimize modifications to the SDI-12 part of the code.  Merely add I2C-slave capability.  

Recommend register sets to cover the various SDI-12 commands.


