Z80 Computer on Spartan 3E Starter Kit

Arquitecture
------------

ROT_CENTER push button (Knob) - Reserved - Reset

Memory
------
0000H - 01FFH - ROM		(0 	- 8191)
2000H - 3FFFH - VIDEO RAM	(8192 - 16383)
4000H - 7FFFH - RAM memory      (16384 - 32767)
8000H - 801FH - LCD Video RAM   (32768 - 32769)

IO
--

Output Ports
------------
01H	- Green Leds (7-0)

Inputs
------

Switches
--------
20H	- SW(3-0)

Push Buttons
------------
30H	- KEY(3-0)

Rotary Knob
-----------
70H	- Rotary control direction 

Keyboard
--------
80H	- Read Keyboard Ascii code

--

Reference Sample ROM

It is provided a ROM with a reference application, and the corresponding
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
	Rotary Knob

The program starts waiting for keys to be typed in the keyboard.
The characters are shown on the 40x30 video (VGA).
If "A" is pressed, then the program starts another routine,
that will write bytes into RAM. After 255 bytes are written,
the bytes are read sequencially and the byte read displayed in the leds (binary format).
When finished, waits for KEY0 (East on S3E board) and then restart again.

The switches (4-0) are used as input to calculate delays. Try
changing these switches to speed up ou slow down the leds and the LCD text rotation speed.
When you change these switches, it will only take effect after a Z80 reset (press the Knob button).

The Rotary knob can be used at any time to rotate the text in the LCD to the left or to the right.

To reset the Z80, use Key West (left push button).

Hope you enjoy.

TO-DO:
----

- 80x40 Video display
- Serial communication
- Monitor program to allow download of programs by serial communication
- Mass storage device (SD/MMC)
- Video colors