@echo off
@echo Programming for SparkFun Qwiic Four Wire Fan Controller
@pause
:loop

@echo -
@echo Flashing bootloader...
rem The fuse bits are *extremely* important to get correct. Be careful.
rem We program these bits separate from flash so that ATtiny is running at 8MHz internal for max flashing programming speed
rem These were found by using ATtiny core and 'Programming Bootloader' with BOD set to 2.7V (required for EEPROM operation)
rem and BOD enabled for active and sleep. Looking at verbose Arduino IDE output:
@avrdude -C avrdude.conf -pattiny841 -cusbtiny -e -Uefuse:w:0b11110100:m -Uhfuse:w:0b11010101:m -Ulfuse:w:0xE2:m

@timeout 1

@echo -
@echo Flashing firmware...

rem The -B1 option reduces the bitclock period (1us = 1MHz SPI), decreasing programming time
rem May increase verification errors

@echo Flashing bootloader and  firmware...
@avrdude -C avrdude.conf -pattiny841 -cusbtiny -e -Uflash:w:Qwiic_Fan_Controller-v10.hex:i

@echo Done programming! Ready for next board.
@pause

goto loop