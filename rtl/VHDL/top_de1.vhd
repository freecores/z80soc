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
--	08 Green Leds		Out		(Port 0x01h)
--	08 Red Leds			Out		(Port 0x02h)
--	04 Seven Seg displays	Out		(Ports 0x10h and 0x11h)
--	36 Pins GPIO0 		In/Out	(Ports 0xA0h, 0xA1h, 0xA2h, 0xA3h, 0xA4h, 0xC0h)
--	36 Pins GPIO1 		In/Out	(Ports 0xB0h, 0xB1h, 0xB2h, 0xB3h, 0xB4h, 0xC1h)
--	08 Switches			In		(Port 0x20h)
--	04 Push buttons		In		(Port 0x30h)
--	PS/2 keyboard 		In		(Port 0x80h)
--	Video Out 40x30 (VGA)	Out		(0x2000h - 0x24B0)
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

entity 	TOP_DE1 is
	port(

    -- Clocks
    CLOCK_27,                                      -- 27 MHz
    CLOCK_50,                                      -- 50 MHz
    EXT_CLOCK : in std_logic;                      -- External Clock

    -- Buttons and switches
    KEY : in std_logic_vector(3 downto 0);         -- Push buttons
    SW : in std_logic_vector(9 downto 0);          -- Switches

    -- LED displays
    HEX0, HEX1, HEX2, HEX3                         -- 7-segment displays
			: out std_logic_vector(6 downto 0);
    LEDG : out std_logic_vector(7 downto 0);       -- Green LEDs
    LEDR : out std_logic_vector(9 downto 0);       -- Red LEDs

    -- RS-232 interface
    UART_TXD : out std_logic;                      -- UART transmitter   
    UART_RXD : in std_logic;                       -- UART receiver

    -- IRDA interface

    -- IRDA_TXD : out std_logic;                      -- IRDA Transmitter
    IRDA_RXD : in std_logic;                       -- IRDA Receiver

    -- SDRAM
    DRAM_DQ : inout std_logic_vector(15 downto 0); -- Data Bus
    DRAM_ADDR : out std_logic_vector(11 downto 0); -- Address Bus    
    DRAM_LDQM,                                     -- Low-byte Data Mask 
    DRAM_UDQM,                                     -- High-byte Data Mask
    DRAM_WE_N,                                     -- Write Enable
    DRAM_CAS_N,                                    -- Column Address Strobe
    DRAM_RAS_N,                                    -- Row Address Strobe
    DRAM_CS_N,                                     -- Chip Select
    DRAM_BA_0,                                     -- Bank Address 0
    DRAM_BA_1,                                     -- Bank Address 0
    DRAM_CLK,                                      -- Clock
    DRAM_CKE : out std_logic;                      -- Clock Enable

    -- FLASH
    FL_DQ : inout std_logic_vector(7 downto 0);      -- Data bus
    FL_ADDR : out std_logic_vector(21 downto 0);     -- Address bus
    FL_WE_N,                                         -- Write Enable
    FL_RST_N,                                        -- Reset
    FL_OE_N,                                         -- Output Enable
    FL_CE_N : out std_logic;                         -- Chip Enable

    -- SRAM
    SRAM_DQ : inout std_logic_vector(15 downto 0); -- Data bus 16 Bits
    SRAM_ADDR : out std_logic_vector(17 downto 0); -- Address bus 18 Bits
    SRAM_UB_N,                                     -- High-byte Data Mask 
    SRAM_LB_N,                                     -- Low-byte Data Mask 
    SRAM_WE_N,                                     -- Write Enable
    SRAM_CE_N,                                     -- Chip Enable
    SRAM_OE_N : out std_logic;                     -- Output Enable

    -- SD card interface
    SD_DAT : in std_logic;      -- SD Card Data      SD pin 7 "DAT 0/DataOut"
    SD_DAT3 : out std_logic;    -- SD Card Data 3    SD pin 1 "DAT 3/nCS"
    SD_CMD : out std_logic;     -- SD Card Command   SD pin 2 "CMD/DataIn"
    SD_CLK : out std_logic;     -- SD Card Clock     SD pin 5 "CLK"

    -- USB JTAG link
    TDI,                        -- CPLD -> FPGA (data in)
    TCK,                        -- CPLD -> FPGA (clk)
    TCS : in std_logic;         -- CPLD -> FPGA (CS)
    TDO : out std_logic;        -- FPGA -> CPLD (data out)

    -- I2C bus
    I2C_SDAT : inout std_logic; -- I2C Data
    I2C_SCLK : out std_logic;   -- I2C Clock

    -- PS/2 port
    PS2_DAT,                    -- Data
    PS2_CLK : inout std_logic;     -- Clock

    -- VGA output
    VGA_HS,                                             -- H_SYNC
    VGA_VS : out std_logic;                             -- SYNC
    VGA_R,                                              -- Red[3:0]
    VGA_G,                                              -- Green[3:0]
    VGA_B : out std_logic_vector(3 downto 0);           -- Blue[3:0]
   
    -- Audio CODEC
    AUD_ADCLRCK : inout std_logic;                      -- ADC LR Clock
    AUD_ADCDAT : in std_logic;                          -- ADC Data
    AUD_DACLRCK : inout std_logic;                      -- DAC LR Clock
    AUD_DACDAT : out std_logic;                         -- DAC Data
    AUD_BCLK : inout std_logic;                         -- Bit-Stream Clock
    AUD_XCK : out std_logic;                            -- Chip Clock
      
    -- General-purpose I/O
    GPIO_0,                                      -- GPIO Connection 0
    GPIO_1 : inout std_logic_vector(35 downto 0) -- GPIO Connection 1	
);
end TOP_DE1;

architecture rtl of TOP_DE1 is

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

	component rom
	port (
		Clk	: in std_logic;
		A	: in std_logic_vector(15 downto 0);
		D	: out std_logic_vector(7 downto 0));
	end component;

	component Clock_357Mhz
	PORT (
		clock_50Mhz				: IN	STD_LOGIC;
		clock_357Mhz			: OUT	STD_LOGIC);
	end component;
	
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

	component decoder_7seg
	port (
		NUMBER		: in   std_logic_vector(3 downto 0);
		HEX_DISP	: out  std_logic_vector(6 downto 0));
	end component;
	
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
	signal clk25mhz		: std_logic;
	signal clk1hz		: std_logic;
	signal clk10hz		: std_logic;
	signal clk100hz		: std_logic;
	
	signal HEX_DISP0	: std_logic_vector(6 downto 0);
	signal HEX_DISP1	: std_logic_vector(6 downto 0);
	signal HEX_DISP2	: std_logic_vector(6 downto 0);
	signal HEX_DISP3	: std_logic_vector(6 downto 0);

	signal NUMBER0		: std_logic_vector(3 downto 0);
	signal NUMBER1		: std_logic_vector(3 downto 0);	
	signal NUMBER2		: std_logic_vector(3 downto 0);
	signal NUMBER3		: std_logic_vector(3 downto 0);
	
	signal GPIO_0_buf_in	: std_logic_vector(35 downto 0);
	signal GPIO_1_buf_in	: std_logic_vector(35 downto 0);

	signal 	vram_rdaddress_sig	: std_logic_vector(12 downto 0);
	signal 	vram_wraddress_sig	: std_logic_vector(15 downto 0);
	signal 	vram_data_sig		: std_logic_vector(7 downto 0);
	signal 	vram_q_sig			: std_logic_vector(7 downto 0);
	signal 	vram_q_reg			: std_logic_vector(7 downto 0);
	signal  vram_wren_sig		: std_logic;
	signal  vram_rden_sig		: std_logic;
	signal 	vram_rdcycle_count	: std_logic_vector(3 downto 0);
	signal 	vram_wrcycle_count	: std_logic_vector(3 downto 0);
	signal  VRAM_CLOCK			: std_logic;
	
	-- PS/2 Keyboard
	signal ps2_read				: std_logic;
	signal ps2_scan_ready		: std_logic;
	signal ps2_ascii_sig		: std_logic_vector(7 downto 0);
	signal ps2_ascii_reg1		: std_logic_vector(7 downto 0);
	signal ps2_ascii_reg		: std_logic_vector(7 downto 0);
	
begin
	
	HEX0 <= HEX_DISP0;
	HEX1 <= HEX_DISP1;
	HEX2 <= HEX_DISP2;
	HEX3 <= HEX_DISP3;
	
	SRAM_ADDR(15 downto 0) <= A - x"4000" when (A >= x"4000" and MReq_n = '0');
	-- SRAM_ADDR(15 downto 0) <= A - x"4000" when (A >= x"4000" and MReq_n = '0') else A;
	-- this is bad --> SRAM_ADDR(15 downto 0) <= A - x"4000";
	SRAM_DQ(15 downto 8) <= (others => 'Z');
	SRAM_ADDR(17 downto 16) <= "00";
	SRAM_UB_N <= '1';
	SRAM_LB_N <= '0';
	SRAM_CE_N <= '0';
	SRAM_WE_N <= Wr_n or MReq_n when A >= x"4000";
	SRAM_OE_N <= Rd_n;
	
	-- Write to SRAM (0x4000 - 0xFFFF)
	SRAM_DQ(7 downto 0) <= DO_CPU when (Wr_n = '0' and MReq_n = '0' and A >= x"4000") else (others => 'Z');

	-- Write into VRAM
	-- this is almost ok -->vram_wraddress_sig <= A - x"2000" when (A >= x"2000" and A < x"4000" and MReq_n = '0' and IORQ_n = '1');
	vram_wraddress_sig <= A - x"2000" when (A >= x"2000" and A < x"4000" and MReq_n = '0');
	-- vram_wraddress_sig <= A - x"2000";
	vram_wren_sig <= not Wr_n when (A >= x"2000" and A < x"4000" and IORQ_n = '1');
	vram_data_sig <= DO_CPU  when (Wr_n = '0' and MReq_n = '0' and A >= x"2000" and A < x"4000") else (others => 'Z');
	-- this is ok --> vram_data_sig <= DO_CPU;
		
	-- Input to Z80
	DI_CPU <= SRAM_DQ(7 downto 0) when (Rd_n = '0' and MReq_n = '0' and A >= x"4000") else 
			-- vram_q_sig when (A >= x"2000" and A < x"4000") else
			D_ROM when (Rd_n = '0' and MReq_n = '0' and A < x"2000") else
			SW(7 downto 0) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"20") else
			("0000" & KEY) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"30") else
			GPIO_0(7 downto 0) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"A0") else
			GPIO_0(15 downto 8) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"A1") else
			GPIO_0(23 downto 16) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"A2") else
			GPIO_0(31 downto 24) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"A3") else
			("0000" & GPIO_0(35 downto 32)) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"A4") else
			GPIO_1(7 downto 0) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"B0") else
			GPIO_1(15 downto 8) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"B1") else
			GPIO_1(23 downto 16) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"B2") else
			GPIO_1(31 downto 24) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"B3") else
			("0000" & GPIO_1(35 downto 32)) when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"B4") else
			ps2_ascii_reg when (IORQ_n = '0' and Rd_n = '0' and A(7 downto 0) = x"80");
	
	-- Process to latch leds and hex displays
	process(Clk_Z80)
	variable NUMBER0_sig	: std_logic_vector(3 downto 0);
	variable NUMBER1_sig	: std_logic_vector(3 downto 0);	
	variable NUMBER2_sig	: std_logic_vector(3 downto 0);
	variable NUMBER3_sig	: std_logic_vector(3 downto 0);
	variable LEDG_sig		: std_logic_vector(7 downto 0);
	variable LEDR_sig		: std_logic_vector(9 downto 0);
	
	variable GPIO_0_buf_out: std_logic_vector(35 downto 0);
	variable GPIO_1_buf_out: std_logic_vector(35 downto 0);
	
	begin	
		
		if Clk_Z80'event and Clk_Z80 = '1' then
		  if IORQ_n = '0' and Wr_n = '0' then
			-- LEDG
			if A(7 downto 0) = x"01" then
				LEDG_sig := DO_CPU;
			-- LEDR
			elsif A(7 downto 0) = x"02" then
				LEDR_sig(7 downto 0) := DO_CPU;
			-- HEX1 and HEX0
			elsif A(7 downto 0) = x"10" then
				NUMBER0_sig := DO_CPU(3 downto 0);
				NUMBER1_sig := DO_CPU(7 downto 4);
			-- HEX3 and HEX2
			elsif A(7 downto 0) = x"11" then
				NUMBER2_sig := DO_CPU(3 downto 0);
				NUMBER3_sig := DO_CPU(7 downto 4);
			-- GPIO_0
			elsif A(7 downto 0) = x"A0" then
				GPIO_0_buf_out(7 downto 0)   := DO_CPU;
			elsif A(7 downto 0) = x"A1" then
				GPIO_0_buf_out(15 downto 8)  := DO_CPU;
			elsif A(7 downto 0) = x"A2" then
				GPIO_0_buf_out(23 downto 16) := DO_CPU;
			elsif A(7 downto 0) = x"A3" then
				GPIO_0_buf_out(31 downto 24) := DO_CPU;
			elsif A(7 downto 0) = x"A4" then
				GPIO_0_buf_out(35 downto 32) := DO_CPU(3 downto 0);
			-- GPIO_1
			elsif A(7 downto 0) = x"B0" then
				GPIO_1_buf_out(7 downto 0)   := DO_CPU;
			elsif A(7 downto 0) = x"B1" then
				GPIO_1_buf_out(15 downto 8)  := DO_CPU;
			elsif A(7 downto 0) = x"B2" then
				GPIO_1_buf_out(23 downto 16) := DO_CPU;
			elsif A(7 downto 0) = x"B3" then
				GPIO_1_buf_out(31 downto 24) := DO_CPU;
			elsif A(7 downto 0) = x"B4" then
				GPIO_1_buf_out(35 downto 32) := DO_CPU(3 downto 0);	
			elsif A(7 downto 0) = x"C0" then
				GPIO_0 <= GPIO_0_buf_out;
			elsif A(7 downto 0) = x"C1" then
				GPIO_1 <= GPIO_1_buf_out;
			end if;
		  end if;
		end if;	
		
		-- Latches the signals
		NUMBER0 <= NUMBER0_sig;
		NUMBER1 <= NUMBER1_sig;
		NUMBER2 <= NUMBER2_sig;
		NUMBER3 <= NUMBER3_sig;
		LEDR(7 downto 0) <= LEDR_sig(7 downto 0);
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
	Rst_n_s <= not SW(9);
	
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

	vga80x40_inst: work.video_80x40 port map (
			CLOCK_50		=> CLOCK_50,
			VRAM_DATA		=> vram_q_sig,
			VRAM_ADDR		=> vram_rdaddress_sig,
			VRAM_CLOCK		=> clk25mhz,
			VRAM_WREN		=> vram_rden_sig,
			VGA_R			=> VGA_R,
			VGA_G			=> VGA_G,
			VGA_B			=> VGA_B,
			VGA_HS			=> VGA_HS,
			VGA_VS			=> VGA_VS
	);

	vram8k_inst : work.vram8k PORT MAP (
		rdaddress	=> vram_rdaddress_sig,
		rdclock	 	=> not clk25mhz,
		rden	 	=> vram_rden_sig,
		q	 		=> vram_q_sig,
		wraddress	=> vram_wraddress_sig(12 downto 0),
		wrclock		=> Clk_Z80,
		wren	 	=> vram_wren_sig,
		data	 	=> vram_data_sig
	);
				
	rom_inst: rom
		port map (
			Clk => Clk_Z80,
			A	=> A,
			D 	=> D_ROM
		);
		
	clkdiv_inst: clk_div
	port map (
		clock_25Mhz				=> clk25mhz,
		clock_1MHz				=> open,
		clock_100KHz			=> open,
		clock_10KHz				=> open,
		clock_1KHz				=> open,
		clock_100Hz				=> clk100hz,
		clock_10Hz				=> clk10hz,
		clock_1Hz				=> clk1hz
	);
		
	clock_z80_inst : Clock_357Mhz
		port map (
			clock_50Mhz		=> CLOCK_50,
			clock_357Mhz	=> Clk_Z80
	);

	DISPHEX0 : decoder_7seg PORT MAP (
		NUMBER			=>	NUMBER0,
		HEX_DISP		=>	HEX_DISP0
	);		

	DISPHEX1 : decoder_7seg PORT MAP (
		NUMBER			=>	NUMBER1,
		HEX_DISP		=>	HEX_DISP1
	);		

	DISPHEX2 : decoder_7seg PORT MAP (
		NUMBER			=>	NUMBER2,
		HEX_DISP		=>	HEX_DISP2
	);		

	DISPHEX3 : decoder_7seg PORT MAP (
		NUMBER			=>	NUMBER3,
		HEX_DISP		=>	HEX_DISP3
	);

	ps2_kbd_inst : work.ps2kbd PORT MAP (
		keyboard_clk	=> PS2_CLK,
		keyboard_data	=> PS2_DAT,
		clock			=> CLOCK_50,
		clkdelay		=> clk100hz,
		reset			=> Rst_n_s,
		read			=> ps2_read,
		scan_ready		=> ps2_scan_ready,
		ps2_ascii_code	=> ps2_ascii_sig
	);
	
	UART_TXD <= 'Z';
	DRAM_ADDR <= (others => '0');
	DRAM_LDQM <= '0';
	DRAM_UDQM <= '0';
	DRAM_WE_N <= '1';
	DRAM_CAS_N <= '1';
	DRAM_RAS_N <= '1';
	DRAM_CS_N <= '1';
	DRAM_BA_0 <= '0';
	DRAM_BA_1 <= '0';
	DRAM_CLK <= '0';
	DRAM_CKE <= '0';
	FL_ADDR <= (others => '0');
	FL_WE_N <= '1';
	FL_RST_N <= '0';
	FL_OE_N <= '1';
	FL_CE_N <= '1';
	TDO <= '0';
	I2C_SCLK <= '0';
	AUD_DACDAT <= '0';
	AUD_XCK <= '0';
	-- Set all bidirectional ports to tri-state
	DRAM_DQ     <= (others => 'Z');
	FL_DQ       <= (others => 'Z');
	I2C_SDAT    <= 'Z';
	AUD_ADCLRCK <= 'Z';
	AUD_DACLRCK <= 'Z';
	AUD_BCLK    <= 'Z';
	GPIO_0 <= (others => 'Z');
	GPIO_1 <= (others => 'Z');	
end;