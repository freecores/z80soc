Z80 Computer on Spartan 3E Starter Kit

Arquitecture
------------

Button West (left) - Reserved - Reset

Memory
------
0000H - 01FFH - ROM		(0 	- 8191)
2000H - 27FFH - VIDEO RAM	(8192  	- 10239)
3FE0H - 3FFFH - LCD Video RAM   (16352 	- 16383)
4000H - 7FFFH - RAM memory      (16384 	- 32763)

IO
--

Output Ports
------------
01H	- Green Leds (7-0)

Inputs
------
20H	- SW(3-0)

30H	- KEY(3-0)

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

The program starts waiting for keys to be typed in the keyboard.
The characters are shown on the 40x30 video (VGA).
If "A" is pressed, then the program starts another routine,
that will write bytes into RAM. After 255 bytes are written,
the bytes are read sequencially and the byte read displayed in the leds (binary format).
When finished, waits for KEY0 (East on S3E board) and then restart again.

The switches (4-0) are used as input to calculate delays. Try
changing these switches to speed up ou slow down the leds. It only
takes effect after a Z80 reset. 
To reset the Z80, use Key West (left push button).

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