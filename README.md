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
Note: SDI/SDO/SSI/SSO are signals used for Ultra-fast mode.
- SDI: Serial Data Input, connected to WRITE. Also used to acknowledge Arduino to set the next data bit when MZ reads. 
- SDO: Serial Data Output, connected to READ. Also used to acknowledge MZ to set the next data bit when Arduino reads.
- SSI: Serial Synchronisation Input, connected to MOTOR. Only used when MZ writes.
- SSO: Serial Synchronisation Output, connected to SENSE. Only used when MZ reads.


## Ultrafast mode

An Ultra-fast mode is provided with MZF-like files to allow around 20000 baud transfer. You must press `RIGHT` button to toggle Ultra-fast mode (disabled by default).  

The following picture is showing the ultra-fast protocol when MZ reads:
![ultra-fast mode](https://user-images.githubusercontent.com/56785/43679133-2cf8ead4-9820-11e8-97a8-876965b69e71.jpg)

Legend: `*` means the MZ reads the READ bit and `><` means Arduino is setting the next READ bit.
Notes:
- CS is also named SSO.
- PC5 is connected to SDO as the current bit data when MZ reads.
- PC1 is connected to SDI as a read acknowledge sent by MZ.
- PC4 is ignored by Arduino when MZ reads. 


## Function


SD CARD BROWSER

───────────────


▲ / ▼       Move through the list

SELECT      Open folder / select file

LEFT        Go back one folder

LEFT hold   Return to SD card root "/"


Supported playback files:
WAV, LEP, L16, MZF, MZT, M12



MENU

────


Hold SELECT   PLAY MENU

Hold RIGHT    RECORD MENU

▲ / ▼         Move through menu items

SELECT        Change selected option

LEFT          Return to browser



PLAYBACK

────────


SELECT        Start / pause / resume

LEFT          Stop playback and return to browser

PLAY CTRL:

MOTOR         Start and pause controlled by MZ MOTOR signal

MANUAL        Controlled only by SELECT button

PLAY MODE:

NORMAL        Standard playback

ULTRA FAST    Fast loader — currently experimental

Important:
 
- INVERT SIG. works for WAV, LEP and L16, not for MZF, MZT, M12



RECORDING

─────────


RIGHT         Start recording using current settings

SELECT        Pause / resume

LEFT          Save recording and return

Hold LEFT     Cancel and delete current recording


REC MODE:

MOTOR         Start/pause controlled by MOTOR signal

AUTO          Starts on WRITE activity, stops after inactivity

MANUAL        Recording starts immediately

REC TYPE:

WAV 22kHz     Smaller WAV file

WAV 44kHz     Higher-quality WAV

LEP 50us      Edge recording

L16 16us      Higher-resolution edge recording



RECORDED FILES

──────────────


Saved in:

/RECORDINGS

Examples:

REC0001.WAV

REC0002.LEP

REC0003.L16

Numbering is shared by all recording types.



DISPLAY

───────


WAIT MOTOR    Waiting for MOTOR signal from MZ

WAIT SIGNAL   AUTO mode waiting for WRITE activity

PLY 042%      Playback in progress

PAU 042%      Playback paused

REC 01:23     Recording in progress

SAVING        Saving file

SAVED         Finished

ERR ...       Error
