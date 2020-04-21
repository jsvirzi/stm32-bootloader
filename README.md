# stm32-bootloader

This code implements enough of AN2606 and AN
Written in C, not C++, to port into embedded applications.
I have designs where one processor boots many processors in the system,
and serial ports (potentially SPI or I2C) are the most straightforward way to update firmware in these designs.

## Notes from the Underground

The STM system bootloader has not failed me yet.
In my opinion, one shortcoming is the documented relationship between flash pages and the system bootloader.
The concept of a flash pages, for the purposes of erasing, appears to be a block of 256 bytes.

## Hex Reader

I implement a poor man's hex reader.
It makes some assumptions about the structure of the hex file, which is not guaranteed to hold.
For example, I assume addresses are contiguously increasing.
Gaps in addressing close out the current "record", 
assuming there is no additional information elsewhere that would fill out the record to capacity.
There is no checking that alignment of hex records are appropriate for the processor.
For example, the STM32H745 requires alignment on 32-byte boundaries.
This hex reader does not check that individual records respect those boundaries.
