-------------------------------------------------------------------------------------------------
-- Z80_Soc (Z80 System on Chip)
-- 
-- Version 0.5 Beta
--
-- Developer: Ronivon Candido Costa
-- Release Date: 2008 / 04 / 16
--
-- Based on the T80 core: http://www.opencores.org/projects.cgi/web/t80
-- This version developed and tested on: Altera DE1 Development Board
--
-- Peripherals configured (Using Ports):
--
--	08 KB Internal ROM	Read		(0x0000h - 0x1FFFh)
--	08 KB INTERNAL VRAM	Write		(0x2000h - 0x3FFFh)
-- 	48 KB External SRAM	Read/Write	(0x4000h - 0xFFFFh)
--	08 Green Leds			Out		(Port 0x01h)
--	08 Red Leds				Out		(Port 0x02h)
--	04 Seven Seg displays	Out		(Ports 0x10h and 0x11h)
--	36 Pins GPIO0 			In/Out	(Ports 0xA0h, 0xA1h, 0xA2h, 0xA3h, 0xA4h, 0xC0h)
--	36 Pins GPIO1 			In/Out	(Ports 0xB0h, 0xB1h, 0xB2h, 0xB3h, 0xB4h, 0xC1h)
--	08 Switches				In		(Port 0x20h)
--	04 Push buttons			In		(Port 0x30h)
--	PS/2 keyboard 			In		(Port 0x80h)
--	Video Out (VGA)			Out		(0x2000h - 0x24B0)
--
--
--  Revision history:
--
-- 2008/04(21 - Ported to Spartan 3E
--
--	2008/04/17 - Added Video support for 80x40 mode
--  2008/04/16 - Release Version 0.5-DE1-Beta
--
-- TO-DO:
-- 	- Monitor program to introduce Z80 Assmebly codes and run
--	- Serial communication, to download assembly code from PC
--	- Add hardware support for 80x40 Video out
--	- SD/MMC card interface to read/store data and programs
-------------------------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;

entity Z80SOC_TOP is
  generic (
    swcount		: integer := 4;
	 keycount	: integer := 4;
	 ledrcount	: integer := 10;
	 ledgcount	: integer := 8;
	 sramdepth	: integer := 16;
	 dramdepth	: integer := 13;
	 framdepth	: integer := 25;
	 vgadepth	: integer := 1);
	port(
    -- Clocks
    CLOCK_50 : in std_logic;                                    -- 50 MHz

    -- Buttons and switches
    KEY : in std_logic_vector(keycount - 1 downto 0);         -- Push buttons
    SW : in std_logic_vector(swcount-1 downto 0);          -- Switches

    -- LED displays
    LEDG : out std_logic_vector(ledgcount-1 downto 0);       -- Green LEDs

    -- RS-232 interface
    -- UART_TXD : out std_logic;                      -- UART transmitter   
    -- UART_RXD : in std_logic;                       -- UART receiver

    -- PS/2 port
    PS2_DAT,                    -- Data
    PS2_CLK : inout std_logic;     -- Clock

    -- VGA output
    VGA_HS,                                             -- H_SYNC
    VGA_VS : out std_logic;                             -- SYNC
    VGA_R,                                              -- Red[3:0]
    VGA_G,                                              -- Green[3:0]
    VGA_B : out std_logic;									   -- Blue[3:0]
	 SF_D  : out std_logic_vector(3 downto 0);
	 LCD_E, LCD_RS, LCD_RW, SF_CE0 : out std_logic 
);
end Z80SOC_TOP;

architecture rtl of Z80SOC_TOP is

	component clk_div
	PORT
	(
		clock_25Mhz				: IN	STD_LOGIC;
		clock_1MHz				: OUT	STD_LOGIC;
		clock_100KHz			: OUT	STD_LOGIC;
		clock_10KHz				: OUT	STD_LOGIC;
		clock_1KHz				: OUT	STD_LOGIC;
		clock_100Hz				: OUT	STD_LOGIC;
		clock_10Hz				: OUT	STD_LOGIC;
		clock_1Hz				: OUT	STD_LOGIC);
	end component;
	
	component T80s
	generic(
		Mode : integer := 0);
	port (
		RESET_n		: in std_logic;
		CLK_n		: in std_logic;
		WAIT_n		: in std_logic;
		INT_n		: in std_logic;
		NMI_n		: in std_logic;
		BUSRQ_n		: in std_logic;
		M1_n		: out std_logic;
		MREQ_n		: out std_logic;
		IORQ_n		: out std_logic;
		RD_n		: out std_logic;
		WR_n		: out std_logic;
		RFSH_n		: out std_logic;
		HALT_n		: out std_logic;
		BUSAK_n		: out std_logic;
		A			: out std_logic_vector(15 downto 0);
		DI			: in std_logic_vector(7 downto 0);
		DO			: out std_logic_vector(7 downto 0));
	end component;

	component lcd
	port(
		clk, reset : in std_logic;
		SF_D : out std_logic_vector(3 downto 0);
		LCD_E, LCD_RS, LCD_RW, SF_CE0 : out std_logic;
		lcd_addr	: out std_logic_vector(4 downto 0);
		lcd_char	: in std_logic_vector(7 downto 0));
	end component;

	component lcdvram
	port (
		addra: IN std_logic_VECTOR(4 downto 0);
		addrb: IN std_logic_VECTOR(4 downto 0);
		clka: IN std_logic;
		clkb: IN std_logic;
		dina: IN std_logic_VECTOR(7 downto 0);
		doutb: OUT std_logic_VECTOR(7 downto 0);
		wea: IN std_logic);
	end component;

	component rom
	port (
		clk	: in std_logic;
		addr	: in std_logic_vector(12 downto 0);
		dout	: out std_logic_vector(7 downto 0));
	end component;

	component Clock_357Mhz
	PORT (
		clock_50Mhz				: IN	STD_LOGIC;
		clock_357Mhz			: OUT	STD_LOGIC);
	end component;
	
	component ps2kbd
	PORT (	
			keyboard_clk	: inout std_logic;
			keyboard_data	: inout std_logic;
			clock			: in std_logic;
			clkdelay		: in std_logic;
			reset			: in std_logic;
			read			: in std_logic;
			scan_ready		: out std_logic;
			ps2_ascii_code	: out std_logic_vector(7 downto 0));
	end component;
	 
component vram
	port (
	addra: IN std_logic_VECTOR(10 downto 0);
	addrb: IN std_logic_VECTOR(10 downto 0);
	clka: IN std_logic;
	clkb: IN std_logic;
	dina: IN std_logic_VECTOR(7 downto 0);
	doutb: OUT std_logic_VECTOR(7 downto 0);
	wea: IN std_logic);
end component;

	COMPONENT video_80x40
	PORT(	CLOCK_50		: IN STD_LOGIC;
			VRAM_DATA		: IN STD_LOGIC_VECTOR(7 DOWNTO 0);
			VRAM_ADDR		: OUT STD_LOGIC_VECTOR(12 DOWNTO 0);
			VRAM_CLOCK		: OUT STD_LOGIC;
			VRAM_WREN		: OUT STD_LOGIC;
			VGA_R,
			VGA_G,
			VGA_B			: OUT STD_LOGIC;
			VGA_HS,
			VGA_VS			: OUT STD_LOGIC);
	END COMPONENT;

	signal MREQ_n	: std_logic;
	signal IORQ_n	: std_logic;
	signal RD_n		: std_logic;
	signal WR_n		: std_logic;
	signal MWr_n	: std_logic;
	signal Rst_n_s	: std_logic;
	signal Clk_Z80	: std_logic;
	signal DI_CPU	: std_logic_vector(7 downto 0);
	signal DO_CPU	: std_logic_vector(7 downto 0);
	signal A		: std_logic_vector(15 downto 0);
	signal One		: std_logic;
	
	signal D_ROM	: std_logic_vector(7 downto 0);

	signal clk25mhz_sig : std_logic;
	signal clk100hz		: std_logic;
	signal clk10hz		: std_logic;
	signal clk1hz		: std_logic;

	signal vram_addra_sig		: std_logic_vector(15 downto 0);
	signal vram_addrb_sig		: std_logic_vector(15 downto 0);
	signal vram_addrb_sigv		: std_logic_vector(15 downto 0);
	signal vram_dina_sig			: std_logic_vector(7 downto 0);
	signal vram_dinb_sig			: std_logic_vector(7 downto 0);
	signal vram_douta_sig		: std_logic_vector(7 downto 0);
	signal vram_doutb_sig		: std_logic_vector(7 downto 0);
	signal vram_doutb_sigv		: std_logic_vector(7 downto 0);
	signal vram_wea_sig			: std_logic;
	signal vram_web_sig			: std_logic;
	signal vram_clka_sig			: std_logic;
	signal vram_clkb_sig			: std_logic;
	signal vram_clkb_sigv			: std_logic;
	
	-- LCD signals
	signal lcd_wea			: std_logic;
	signal lcd_addra		: std_logic_vector(4 downto 0);
	signal lcd_addrb		: std_logic_vector(4 downto 0);
	signal lcd_dina		: std_logic_vector(7 downto 0);
	signal lcd_doutb		: std_logic_vector(7 downto 0);
	
	-- VGA conversion from 4 bits to 8 bit
	signal VGA_Rs, VGA_Gs, VGA_Bs	: std_logic_vector(3 downto 0);
	signal VGA_HSs, VGA_VSs 		: std_logic;
	
	-- PS/2 Keyboard
	signal ps2_read					: std_logic;
	signal ps2_scan_ready			: std_logic;
	signal ps2_ascii_sig				: std_logic_vector(7 downto 0);
	signal ps2_ascii_reg1			: std_logic_vector(7 downto 0);
	signal ps2_ascii_reg				: std_logic_vector(7 downto 0);
	
begin
	
	Rst_n_s <= not KEY(3);

	writevram: process(Clk_Z80)
	begin
		if Clk_Z80'event and Clk_Z80 = '1' then	
			if A >= x"2000" and A <= x"27FF" then
				vram_addra_sig <= A - x"2000";
				if Wr_n = '0' and MReq_n = '0' then
					vram_dina_sig <= DO_CPU;
					vram_wea_sig <= '0';
				else
					vram_wea_sig <= '1';
				end if;
			else
				vram_wea_sig <= '1';
			end if;
		end if;
	end process;
	
	lcd_process: process(Clk_Z80)
	begin	
		if Clk_Z80'event and Clk_Z80 = '1' then
			if A >= x"3FE0" and A < x"4000" then
				if MReq_n = '0' and Wr_n = '0' then
					lcd_wea <= '0';
					lcd_addra <= A - x"3FE0";
					lcd_dina <= DO_CPU;
				else
					lcd_wea <= '1';
				end if;
			else
				lcd_wea <= '1';
		  end if;
		end if;
	end process;
	
	DI_CPU <= D_ROM when (Rd_n = '0' and MReq_n = '0' and A < x"2000") else
			("0000" & SW) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"20") else
			("0000" & KEY) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"30") else
			ps2_ascii_reg when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"80");
		
	-- Process to latch leds and hex displays
	process(Clk_Z80)
	variable LEDG_sig		: std_logic_vector(7 downto 0);
	
	begin	
		
		if Clk_Z80'event and Clk_Z80 = '1' then
		  if IORQ_n = '0' and Wr_n = '0' then
			-- LEDG
			if A(7 downto 0) = x"01" then
				LEDG_sig := DO_CPU;
			end if;
		  end if;
		end if;	
		
		-- Latches the signals
		LEDG <= LEDG_sig;
	end process;		
	
	-- the following three processes deals with different clock domain signals
	ps2_process1: process(CLOCK_50)
	begin
		if CLOCK_50'event and CLOCK_50 = '1' then
			if ps2_read = '1' then
				if ps2_ascii_sig /= x"FF" then
					ps2_read <= '0';
					ps2_ascii_reg1 <= "00000000";
				end if;
			elsif ps2_scan_ready = '1' then
				if ps2_ascii_sig = x"FF" then
					ps2_read <= '1';
				else
					ps2_ascii_reg1 <= ps2_ascii_sig;
				end if;
			end if;
		end if;
	end process;
	
	ps2_process2: process(Clk_Z80)
	begin
		if Clk_Z80'event and Clk_Z80 = '1' then
			ps2_ascii_reg <= ps2_ascii_reg1;
		end if;
	end process;
	
	
	One <= '1';
	z80_inst: T80s
		port map (
			M1_n => open,
			MREQ_n => MReq_n,
			IORQ_n => IORq_n,
			RD_n => Rd_n,
			WR_n => Wr_n,
			RFSH_n => open,
			HALT_n => open,
			WAIT_n => One,
			INT_n => One,
			NMI_n => One,
			RESET_n => Rst_n_s,
			BUSRQ_n => One,
			BUSAK_n => open,
			CLK_n => Clk_Z80,
			A => A,
			DI => DI_CPU,
			DO => DO_CPU
		);
		  
	vga80x40_inst: video_80x40 port map (
			CLOCK_50			=> CLOCK_50,
			VRAM_DATA		=> vram_doutb_sig,
			VRAM_ADDR		=> vram_addrb_sig(12 downto 0),
			VRAM_CLOCK		=> vram_clkb_sig,
			VRAM_WREN		=> vram_web_sig,
			VGA_R				=> VGA_R,
			VGA_G				=> VGA_G,
			VGA_B				=> VGA_B,
			VGA_HS			=> VGA_HS,
			VGA_VS			=> VGA_VS
	);
			
	vram_inst: vram port map (
		clka 		=> Clk_Z80,
		clkb		=> vram_clkb_sig,
      wea    	=> vram_wea_sig,
      addra		=> vram_addra_sig(10 downto 0),
      addrb		=> vram_addrb_sig(10 downto 0),
      dina   	=> vram_dina_sig,
		doutb		=> vram_doutb_sig
	);
			
	rom_inst: rom
		port map (
			clk => Clk_Z80,
			addr	=> A(12 downto 0),
			dout 	=> D_ROM
		);

	ps2_kbd_inst : ps2kbd PORT MAP (
		keyboard_clk	=> PS2_CLK,
		keyboard_data	=> PS2_DAT,
		clock			=> CLOCK_50,
		clkdelay		=> clk100hz,
		reset			=> Rst_n_s,
		read			=> ps2_read,
		scan_ready		=> ps2_scan_ready,
		ps2_ascii_code	=> ps2_ascii_sig
	);
	
	
	process (CLOCK_50)
   begin
		if CLOCK_50'event and CLOCK_50 = '1' then
        clk25mhz_sig <= not clk25mhz_sig;
		end if;
   end process;

		clock_z80_inst : Clock_357Mhz
		port map (
			clock_50Mhz		=> CLOCK_50,
			clock_357Mhz	=> Clk_Z80
	);
	
   clkdiv_inst: clk_div
		port map (
		clock_25Mhz		=> clk25mhz_sig,		
		clock_1MHz		=> open,
		clock_100KHz	=> open,
		clock_10KHz		=> open,
		clock_1KHz		=> open,
		clock_100Hz		=> clk100hz,	
		clock_10Hz		=> clk10hz,
		clock_1Hz		=> clk1hz
	);	

	lcd_inst: lcd
	port map (
		clk			=> CLOCK_50,
		reset			=> not Rst_n_s,
		SF_D 			=> SF_D,
		LCD_E			=> LCD_E,
		LCD_RS		=> LCD_RS,
		LCD_RW		=> LCD_RW,
		SF_CE0 		=> SF_CE0,
		lcd_addr		=> lcd_addrb,
		lcd_char		=> lcd_doutb
	);
	
	lcdvram_inst : lcdvram
		port map (
			addra => lcd_addra,
			addrb => lcd_addrb,
			clka => Clk_Z80,
			clkb => CLOCK_50,
			dina => lcd_dina,
			doutb => lcd_doutb,
			wea => lcd_wea
		);
			
end;