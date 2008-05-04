library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;

entity rom is
        port(
                clk             : in std_logic;
                addr            : in std_logic_vector(12 downto 0);
                dout            : out std_logic_vector(7 downto 0)
        );
end rom;

architecture rtl of rom is
begin

process (clk)
begin
 if clk'event and clk = '1' then
        case addr is
             when "0000000000000" => dout <= x"3E";
             when "0000000000001" => dout <= x"5A";
             when "0000000000010" => dout <= x"32";
             when "0000000000011" => dout <= x"10";
             when "0000000000100" => dout <= x"20";
             when "0000000000101" => dout <= x"32";
             when "0000000000110" => dout <= x"E0";
             when "0000000000111" => dout <= x"3F";
             when "0000000001000" => dout <= x"3E";
             when "0000000001001" => dout <= x"38";
             when "0000000001010" => dout <= x"32";
             when "0000000001011" => dout <= x"11";
             when "0000000001100" => dout <= x"20";
             when "0000000001101" => dout <= x"32";
             when "0000000001110" => dout <= x"E1";
             when "0000000001111" => dout <= x"3F";
             when "0000000010000" => dout <= x"3E";
             when "0000000010001" => dout <= x"30";
             when "0000000010010" => dout <= x"32";
             when "0000000010011" => dout <= x"12";
             when "0000000010100" => dout <= x"20";
             when "0000000010101" => dout <= x"32";
             when "0000000010110" => dout <= x"E2";
             when "0000000010111" => dout <= x"3F";
             when "0000000011000" => dout <= x"3E";
             when "0000000011001" => dout <= x"20";
             when "0000000011010" => dout <= x"32";
             when "0000000011011" => dout <= x"13";
             when "0000000011100" => dout <= x"20";
             when "0000000011101" => dout <= x"32";
             when "0000000011110" => dout <= x"E3";
             when "0000000011111" => dout <= x"3F";
             when "0000000100000" => dout <= x"3E";
             when "0000000100001" => dout <= x"53";
             when "0000000100010" => dout <= x"32";
             when "0000000100011" => dout <= x"14";
             when "0000000100100" => dout <= x"20";
             when "0000000100101" => dout <= x"32";
             when "0000000100110" => dout <= x"E4";
             when "0000000100111" => dout <= x"3F";
             when "0000000101000" => dout <= x"3E";
             when "0000000101001" => dout <= x"6F";
             when "0000000101010" => dout <= x"32";
             when "0000000101011" => dout <= x"15";
             when "0000000101100" => dout <= x"20";
             when "0000000101101" => dout <= x"32";
             when "0000000101110" => dout <= x"E5";
             when "0000000101111" => dout <= x"3F";
             when "0000000110000" => dout <= x"3E";
             when "0000000110001" => dout <= x"43";
             when "0000000110010" => dout <= x"32";
             when "0000000110011" => dout <= x"16";
             when "0000000110100" => dout <= x"20";
             when "0000000110101" => dout <= x"32";
             when "0000000110110" => dout <= x"E6";
             when "0000000110111" => dout <= x"3F";
             when "0000000111000" => dout <= x"DB";
             when "0000000111001" => dout <= x"30";
             when "0000000111010" => dout <= x"D3";
             when "0000000111011" => dout <= x"01";
             when "0000000111100" => dout <= x"FE";
             when "0000000111101" => dout <= x"01";
             when "0000000111110" => dout <= x"20";
             when "0000000111111" => dout <= x"F8";
             when "0000001000000" => dout <= x"DB";
             when "0000001000001" => dout <= x"30";
             when "0000001000010" => dout <= x"FE";
             when "0000001000011" => dout <= x"01";
             when "0000001000100" => dout <= x"28";
             when "0000001000101" => dout <= x"FA";
             when "0000001000110" => dout <= x"DB";
             when "0000001000111" => dout <= x"20";
             when "0000001001000" => dout <= x"D3";
             when "0000001001001" => dout <= x"01";
             when "0000001001010" => dout <= x"26";
             when "0000001001011" => dout <= x"3F";
             when "0000001001100" => dout <= x"2E";
             when "0000001001101" => dout <= x"F0";
             when "0000001001110" => dout <= x"DB";
             when "0000001001111" => dout <= x"80";
             when "0000001010000" => dout <= x"B7";
             when "0000001010001" => dout <= x"28";
             when "0000001010010" => dout <= x"FB";
             when "0000001010011" => dout <= x"77";
             when "0000001010100" => dout <= x"2C";
             when "0000001010101" => dout <= x"7D";
             when "0000001010110" => dout <= x"FE";
             when "0000001010111" => dout <= x"00";
             when "0000001011000" => dout <= x"20";
             when "0000001011001" => dout <= x"F4";
             when "0000001011010" => dout <= x"18";
             when "0000001011011" => dout <= x"EA";
             when others => dout <= x"00";
        end case;
 end if;
end process;
end;
