 Reverse engineering a Konnwei KW206 OBD2 https://www.konnwei.com/product/430.html to read Tesla Model S data from the canbus.
 
 CANBUS packets for Model S/X decoded by Jason Hughes  https://skie.net/ used in the sketch.







AT32F415 MCU


MCU PINS


Programming

SWDIO → PA13 → QFN48 pin 34 (pin2)

SWCLK → PA14 → QFN48 pin 37 (pin3)

NRST → QFN48 pin 7

BOOT0 → QFN48 pin 44 for recovery/boot-mode work if needed.


I/O

PB6 pin 42 - down button

PA13 pin 34 - up button

PA14 pin 37 - esc button

PB2 pin 20 - ok button

PB3 pin 39 - buzzer

PB1 pin 19 - photocell

PB0 pin 18 - input power coming in to MCU ADC (analog input)

PB7 pin 43 - USB power status - low for yes, high for no

PA8 pin 29 - 12v power status - high for yes, low for no



EN25Q64 - eeprom

PB12 pin 25 - CS# pin 1

PB13 pin 26 -  CLK pin 6

PB14 pin 27 - DO (DQ1) pin 2

PB15 pin 28 - DI (DQ0) pin 5





CA-IF1051H - can transceiver

PA12 pin 33 - TXD pin 1 (Micro USB)

PA11 pin 32 - RXD pin 4 (Micro USB)







JLT35002A-PSS - 320x480 8-bit parallel LCD

MCU pin - LCD pin mapping

PA15 pin 38 - 3 PWM backlight brightness

PB4 pin 40 - 8 reset lcd

PC13 pin 2 - pin 34 WR/SCL

PC15 pin 4 - pin 35 RS


LCD DATA

PA7 pin 17 - pin 23

PA6 pin 16 - pin 24

PA5 pin 15 - pin 25

PA4 pin 14 - pin 26

PA3 pin 13 - pin 27

PA2 pin 12 - pin 28

PA1 pin 11 - pin 29

PA0 pin 10 - pin 30


J1850 PWM pin (untested)

PB5 pin 41 - left side GREEN (J1850 PWM)

PB8 pin 45 - left side WHITE (J1850 PWM)

PB9 pin 46 - pin 7 - LM393 (left) output (J1850 PWM)

PC14 pin 3 - left side RED (J1850 PWM)

PB11 pin 22 - pin 1 - LM393 (right) output (J1850 PWM)

PB10 pin 21 - right side BLUE (J1850 PWM)

PA10 pin 31 - right side YELLOW (J1850 PWM)
