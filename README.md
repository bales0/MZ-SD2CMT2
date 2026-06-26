# MZ-SD2CMT2 - Reborn

Based on work and ideas of hlide and Daniel Coulom to improve UI and features of original MZ-SD2CMT.

!!!WORK IN PROGRESS!!!

SD card based CMT Emulator for Sharp MZ-800, supporting playing WAV, LEP, MZF and recording WAV, LEP.

## Parts Used
1. Arduino Mini MEGA 2560
2. SD card module with level shifter inbuilt
3. 16x2 4-bit parallel LCD with 5 x Buttons

- an Arduino MEGA 2560 R3 ATMEGA 16U2 could be also used:
![71iRODIgxUL _SL1500_](https://user-images.githubusercontent.com/56785/55276248-0c186780-52f2-11e9-8fc0-8e57b2302ac0.jpg)

- a 16x2 LCD with 5 buttons:
![71By4b4xsRL _SL1200_](https://user-images.githubusercontent.com/56785/55276357-30287880-52f3-11e9-9958-61ec5a2bd57e.jpg)

- Dupont Wires to connect the Arduino to the SD card module and the MZ tape connector.


## Installation
You need SdFat, CrystalLiquid, they are available from PlatformIO IDE.

## Wiring

Arduino MEGA Pins:

 Name | Number | Direction | Description                       
 ---- | ------:|:---------:|:-----------
 A0   |        | <-        | BUTTON (UP/DOWN/LEFT/RIGHT/SELECT)
 *A1  |        | <-        | IR remote data line - not supported
 .    | 4      | ->        | LCD D4
 .    | 5      | ->        | LCD D5
 .    | 6      | ->        | LCD D6
 .    | 7      | ->        | LCD D7
 .    | 8      | ->        | LCD RESET
 .    | 9      | ->        | LCD ENABLE
 RX3  | 15     | <-        | MZCMT WRITE
 TX2  | 16     | <-        | MZCMT MOTOR
 OC3B | 2      | ->        | MZCMT READ
 TX1  | 18     | ->        | MZCMT SENSE
 RX1  | 19     | ->        | MZCMT LED
 *SDA | 20     | ->        | OLED SDA (I²C) - not supported
 *SCL | 21     | ->        | OLED SCK (I²C) - not supported
 .    | 50     | <-        | SD MISO (SD Card MISO PIN)
 .    | 51     | ->        | SD MOSI (SD Card MOSI PIN)
 .    | 52     | ->        | SD SCK  (SD Card SCK PIN)
 .    | 53     | ->        | SD SS   (SD Card slave select)


LCD Pins:

 Name | Direction | Connected to                       
 ---- |:---------:|:------------
 D4   | <-        | ARDUINO #4
 D5   | <-        | ARDUINO #5
 D6   | <-        | ARDUINO #6
 D7   | <-        | ARDUINO #7
 RESET| <-        | ARDUINO #8
 VCC  | <-        | ARDUINO 5v
 GND  | <-        | ARDUINO GND


SD CARD Pins:

 Name | Direction | Connected to                       
 ---- |:---------:|:------------
 GND  | <-        | ARDUINO GND
3.3V  | <-        | NC
  5V  | <-        | ARDUINO 5V
MISO  | ->        | ARDUINO #50
MOSI  | <-        | ARDUINO #51
 SCK  | <-        | ARDUINO #52
SDCS  | <-        | ARDUINO #53


Wire up as above, and program the Arduino MEGA using the IDE.

The following picture is showing how to connect Arduino to 8255:  
![mz-700 - 8255 <-> Arduino](https://user-images.githubusercontent.com/56785/47266539-4eb26880-d538-11e8-9fdb-7d2fadc24ca2.png)

Drop some MZF/M12/MZT files onto a FAT32 formatted SD card, plug it into the mz-sd²cmt, and power on.


## Old usage
Drop some LEP or WAV files (converted MZF Files through MZF2LEP tool) onto a FAT32 formatted SD card, plug it into the mz-sd2cmt2, and power on.
Note that WAV files have a limitation: 8-bit mono channel 22kHz and 44kHz.

## Issues

LEP file is supported. Suffixes .LEP and .L16 are for time resolution 50µs and 16µs (As the original LEP from SDLEP-READER - Daniel Coulon). The only interest is for a program needing to read severals blocks. Maybe the same thing can be handled through a MZT file (with multiple data blocks) by listening to MOTOR signal to separate block readings. But unlike LEP, there is no way to say whether the next block is a header block or a data block.

Some programs are a set of blocks in the tape: the first program will read the rest in one or several blocks. Right now, MZF, M12 and MZT don't handle them correctly (no indication whether the next block is a header or a data so you can emit the right prolog). Maybe defining a new binary file with those indication may help to allow reading multiple data. 
