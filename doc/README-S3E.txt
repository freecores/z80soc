Z80 Computer on Spartan 3E Starter Kit

Arquitecture
------------


SW3 	- Reserved - Reset

Memory
------
0000H - 01FFH - ROM		(0 	- 8191)
2000H - 27FFH - VIDEO RAM	(8192  	- 10239)
3FE0H - 3FFFH - LCD Video RAM   (16352 	- 16383)

IO
--

Output Ports
------------
01H	- Green Leds (7-0)

Inputs
------
20H	- SW(3-0)

30H	- KEY(3-0)

LCD
---
3FE0H	- 3FFFh - LCD video memory (32 characters)

Video
-----
2000H	- 27FFH	- Video RAM Memory

Keyboard
--------
80H	- Read Keyboard Ascii code

--

Reference Sample ROM

It is provided a ROM with a reference application, and the correspnding
Z80 source codes.

To use the application you will need to connect the S3E board to a
VGA monitor and PS/2 keyboard.

The program will show how to use:
	Input push buttons
	Input Switches
	PS/2 keyboard
	Video text out
	Leds
	LCD

The ROM provided will wait for key East of the S3E to proceed.
Then, will show the keys type in the keyboard on the second line of the LCD.
The Switches will be displayed in the LEDS when a key is pressed in the keyboard.

Key West on the S3E will reset the system.

Hope you enjoy.


Bugs
----

- Z80 programs that have loops using registers HL, DE, BC are not working. This happens only with S3E port. The DE1 version is working 100% what concerns to z80 programs.

TO-DO:
----

- 80x40 Video display
- Serial communication
- Monitor program to allow download of programs by serial communication
- Mass storage device (SD/MMC)
- Video colors