STM32 BASED STOPWATCH FIRMWARE
====================================


This  project aims  to  create a  firmware to  be  uploaded in  STM32VLDiscovery
device, which  allows to  measure the time  a falling object  falls in  a copper
pipe. Pipe is  fitted with two IR LED and  phototransistor pairs, which initiate
and terminate  the timing. The purpose  of dropping objects through  a copper (a
good conductor) pipe is  to demonstrate in a physics lecture  the effect of eddy
currents - a  permanent magnet falling freely through a  conducting pipe induces
eddy  currents in  the pipe.  These  eddy currents  act to  oppose the  changing
magnetic field, according to Lenz's law,  and thereby slowing down the permanent
magnet.


The timing is based on STM32's TIMER functionality.

Valuable, albeit a bit difficult to read for a newbie, sources of information are [RM0041 Reference manual STM32F100xx advanced ARM-based 32-bit MCUs](http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/REFERENCE_MANUAL/CD00246267.pdf) and [UM0919 User Manual STM32VLDISCOVERY STM32 value line Discovery](http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/USER_MANUAL/CD00267113.pdf).

Compiler for bare-metal ARM can be summoned from [esden's summon-arm-toolchain repo](https://github.com/esden/summon-arm-toolchain).
