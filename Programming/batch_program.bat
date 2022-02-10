@echo Programming the SparkFun Qwiic Blower Fan.
@pause
:loop

@echo -
@echo Flashing bootloader...
rem The fuse bits are *extremely* important to get correct. Be careful.
rem These were found by using ATtiny core and 'Programming Bootloader' and looking at verbose Arduino IDE output
rem This enables B.O.D at 2.7v and sets the clock source to external 16Mhz xtal
@avrdude -C avrdude.conf -pattiny841 -cusbtiny -e -Uefuse:w:0b11111110:m -Uhfuse:w:0b11011101:m -Ulfuse:w:0xEE:m 



@timeout 1

@echo -
@echo Flashing firmware...

@echo Flashing bootloader and  firmware...
@avrdude -C avrdude.conf -pattiny841 -cusbtiny -e -Uflash:w:Qwiic_Fan_Controller_v11_PCFan.hex:i

@echo Done programming!
@pause

goto loop