library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;

entity rom is
        port(
                Clk             : in std_logic;
                A               : in std_logic_vector(11 downto 0);
                D               : out std_logic_vector(7 downto 0)
        );
end rom;

architecture rtl of rom is
begin

process (Clk)
begin
 if Clk'event and Clk = '1' then
        case A is
             when x"000" => D <= x"31";
             when x"001" => D <= x"FE";
             when x"002" => D <= x"BF";
             when x"003" => D <= x"3A";
             when x"004" => D <= x"DF";
             when x"005" => D <= x"FF";
             when x"006" => D <= x"B7";
             when x"007" => D <= x"28";
             when x"008" => D <= x"03";
             when x"009" => D <= x"CD";
             when x"00A" => D <= x"95";
             when x"00B" => D <= x"01";
             when x"00C" => D <= x"CD";
             when x"00D" => D <= x"F7";
             when x"00E" => D <= x"00";
             when x"00F" => D <= x"CD";
             when x"010" => D <= x"0B";
             when x"011" => D <= x"02";
             when x"012" => D <= x"11";
             when x"013" => D <= x"34";
             when x"014" => D <= x"40";
             when x"015" => D <= x"21";
             when x"016" => D <= x"17";
             when x"017" => D <= x"02";
             when x"018" => D <= x"CD";
             when x"019" => D <= x"0A";
             when x"01A" => D <= x"01";
             when x"01B" => D <= x"11";
             when x"01C" => D <= x"5C";
             when x"01D" => D <= x"40";
             when x"01E" => D <= x"21";
             when x"01F" => D <= x"2A";
             when x"020" => D <= x"02";
             when x"021" => D <= x"CD";
             when x"022" => D <= x"0A";
             when x"023" => D <= x"01";
             when x"024" => D <= x"CD";
             when x"025" => D <= x"15";
             when x"026" => D <= x"01";
             when x"027" => D <= x"11";
             when x"028" => D <= x"46";
             when x"029" => D <= x"41";
             when x"02A" => D <= x"21";
             when x"02B" => D <= x"3D";
             when x"02C" => D <= x"02";
             when x"02D" => D <= x"CD";
             when x"02E" => D <= x"0A";
             when x"02F" => D <= x"01";
             when x"030" => D <= x"11";
             when x"031" => D <= x"96";
             when x"032" => D <= x"41";
             when x"033" => D <= x"21";
             when x"034" => D <= x"59";
             when x"035" => D <= x"02";
             when x"036" => D <= x"CD";
             when x"037" => D <= x"0A";
             when x"038" => D <= x"01";
             when x"039" => D <= x"11";
             when x"03A" => D <= x"E6";
             when x"03B" => D <= x"41";
             when x"03C" => D <= x"21";
             when x"03D" => D <= x"76";
             when x"03E" => D <= x"02";
             when x"03F" => D <= x"CD";
             when x"040" => D <= x"0A";
             when x"041" => D <= x"01";
             when x"042" => D <= x"11";
             when x"043" => D <= x"36";
             when x"044" => D <= x"42";
             when x"045" => D <= x"21";
             when x"046" => D <= x"93";
             when x"047" => D <= x"02";
             when x"048" => D <= x"CD";
             when x"049" => D <= x"0A";
             when x"04A" => D <= x"01";
             when x"04B" => D <= x"11";
             when x"04C" => D <= x"5E";
             when x"04D" => D <= x"42";
             when x"04E" => D <= x"21";
             when x"04F" => D <= x"A7";
             when x"050" => D <= x"02";
             when x"051" => D <= x"CD";
             when x"052" => D <= x"0A";
             when x"053" => D <= x"01";
             when x"054" => D <= x"21";
             when x"055" => D <= x"4A";
             when x"056" => D <= x"03";
             when x"057" => D <= x"3A";
             when x"058" => D <= x"DF";
             when x"059" => D <= x"FF";
             when x"05A" => D <= x"B7";
             when x"05B" => D <= x"28";
             when x"05C" => D <= x"03";
             when x"05D" => D <= x"21";
             when x"05E" => D <= x"56";
             when x"05F" => D <= x"03";
             when x"060" => D <= x"11";
             when x"061" => D <= x"87";
             when x"062" => D <= x"40";
             when x"063" => D <= x"CD";
             when x"064" => D <= x"0A";
             when x"065" => D <= x"01";
             when x"066" => D <= x"11";
             when x"067" => D <= x"D6";
             when x"068" => D <= x"42";
             when x"069" => D <= x"01";
             when x"06A" => D <= x"1E";
             when x"06B" => D <= x"00";
             when x"06C" => D <= x"C5";
             when x"06D" => D <= x"D5";
             when x"06E" => D <= x"CD";
             when x"06F" => D <= x"E2";
             when x"070" => D <= x"00";
             when x"071" => D <= x"CD";
             when x"072" => D <= x"3F";
             when x"073" => D <= x"01";
             when x"074" => D <= x"3A";
             when x"075" => D <= x"DF";
             when x"076" => D <= x"FF";
             when x"077" => D <= x"B7";
             when x"078" => D <= x"28";
             when x"079" => D <= x"03";
             when x"07A" => D <= x"CD";
             when x"07B" => D <= x"72";
             when x"07C" => D <= x"01";
             when x"07D" => D <= x"D1";
             when x"07E" => D <= x"C1";
             when x"07F" => D <= x"CD";
             when x"080" => D <= x"12";
             when x"081" => D <= x"01";
             when x"082" => D <= x"B7";
             when x"083" => D <= x"28";
             when x"084" => D <= x"E7";
             when x"085" => D <= x"FE";
             when x"086" => D <= x"41";
             when x"087" => D <= x"28";
             when x"088" => D <= x"16";
             when x"089" => D <= x"D3";
             when x"08A" => D <= x"01";
             when x"08B" => D <= x"6F";
             when x"08C" => D <= x"3A";
             when x"08D" => D <= x"DF";
             when x"08E" => D <= x"FF";
             when x"08F" => D <= x"B7";
             when x"090" => D <= x"20";
             when x"091" => D <= x"03";
             when x"092" => D <= x"7D";
             when x"093" => D <= x"D3";
             when x"094" => D <= x"11";
             when x"095" => D <= x"7D";
             when x"096" => D <= x"12";
             when x"097" => D <= x"13";
             when x"098" => D <= x"0B";
             when x"099" => D <= x"78";
             when x"09A" => D <= x"B1";
             when x"09B" => D <= x"28";
             when x"09C" => D <= x"C9";
             when x"09D" => D <= x"18";
             when x"09E" => D <= x"CD";
             when x"09F" => D <= x"21";
             when x"0A0" => D <= x"00";
             when x"0A1" => D <= x"80";
             when x"0A2" => D <= x"3E";
             when x"0A3" => D <= x"00";
             when x"0A4" => D <= x"77";
             when x"0A5" => D <= x"23";
             when x"0A6" => D <= x"3C";
             when x"0A7" => D <= x"20";
             when x"0A8" => D <= x"FB";
             when x"0A9" => D <= x"CD";
             when x"0AA" => D <= x"CD";
             when x"0AB" => D <= x"00";
             when x"0AC" => D <= x"21";
             when x"0AD" => D <= x"00";
             when x"0AE" => D <= x"80";
             when x"0AF" => D <= x"3A";
             when x"0B0" => D <= x"DF";
             when x"0B1" => D <= x"FF";
             when x"0B2" => D <= x"B7";
             when x"0B3" => D <= x"20";
             when x"0B4" => D <= x"06";
             when x"0B5" => D <= x"7C";
             when x"0B6" => D <= x"D3";
             when x"0B7" => D <= x"11";
             when x"0B8" => D <= x"7D";
             when x"0B9" => D <= x"D3";
             when x"0BA" => D <= x"10";
             when x"0BB" => D <= x"7E";
             when x"0BC" => D <= x"D3";
             when x"0BD" => D <= x"01";
             when x"0BE" => D <= x"CD";
             when x"0BF" => D <= x"E2";
             when x"0C0" => D <= x"00";
             when x"0C1" => D <= x"23";
             when x"0C2" => D <= x"7E";
             when x"0C3" => D <= x"FE";
             when x"0C4" => D <= x"FF";
             when x"0C5" => D <= x"20";
             when x"0C6" => D <= x"F4";
             when x"0C7" => D <= x"CD";
             when x"0C8" => D <= x"CD";
             when x"0C9" => D <= x"00";
             when x"0CA" => D <= x"C3";
             when x"0CB" => D <= x"03";
             when x"0CC" => D <= x"00";
             when x"0CD" => D <= x"CD";
             when x"0CE" => D <= x"E2";
             when x"0CF" => D <= x"00";
             when x"0D0" => D <= x"DB";
             when x"0D1" => D <= x"30";
             when x"0D2" => D <= x"FE";
             when x"0D3" => D <= x"01";
             when x"0D4" => D <= x"20";
             when x"0D5" => D <= x"F7";
             when x"0D6" => D <= x"C9";
             when x"0D7" => D <= x"DB";
             when x"0D8" => D <= x"20";
             when x"0D9" => D <= x"B7";
             when x"0DA" => D <= x"20";
             when x"0DB" => D <= x"02";
             when x"0DC" => D <= x"3E";
             when x"0DD" => D <= x"01";
             when x"0DE" => D <= x"3D";
             when x"0DF" => D <= x"20";
             when x"0E0" => D <= x"FD";
             when x"0E1" => D <= x"C9";
             when x"0E2" => D <= x"DB";
             when x"0E3" => D <= x"20";
             when x"0E4" => D <= x"B7";
             when x"0E5" => D <= x"20";
             when x"0E6" => D <= x"02";
             when x"0E7" => D <= x"3E";
             when x"0E8" => D <= x"01";
             when x"0E9" => D <= x"F5";
             when x"0EA" => D <= x"01";
             when x"0EB" => D <= x"88";
             when x"0EC" => D <= x"13";
             when x"0ED" => D <= x"0B";
             when x"0EE" => D <= x"78";
             when x"0EF" => D <= x"B1";
             when x"0F0" => D <= x"20";
             when x"0F1" => D <= x"FB";
             when x"0F2" => D <= x"F1";
             when x"0F3" => D <= x"3D";
             when x"0F4" => D <= x"20";
             when x"0F5" => D <= x"F3";
             when x"0F6" => D <= x"C9";
             when x"0F7" => D <= x"3E";
             when x"0F8" => D <= x"00";
             when x"0F9" => D <= x"D3";
             when x"0FA" => D <= x"91";
             when x"0FB" => D <= x"D3";
             when x"0FC" => D <= x"92";
             when x"0FD" => D <= x"01";
             when x"0FE" => D <= x"B0";
             when x"0FF" => D <= x"04";
             when x"100" => D <= x"3E";
             when x"101" => D <= x"20";
             when x"102" => D <= x"D3";
             when x"103" => D <= x"90";
             when x"104" => D <= x"0B";
             when x"105" => D <= x"78";
             when x"106" => D <= x"B1";
             when x"107" => D <= x"20";
             when x"108" => D <= x"F7";
             when x"109" => D <= x"C9";
             when x"10A" => D <= x"7E";
             when x"10B" => D <= x"B7";
             when x"10C" => D <= x"C8";
             when x"10D" => D <= x"12";
             when x"10E" => D <= x"23";
             when x"10F" => D <= x"13";
             when x"110" => D <= x"18";
             when x"111" => D <= x"F8";
             when x"112" => D <= x"DB";
             when x"113" => D <= x"80";
             when x"114" => D <= x"C9";
             when x"115" => D <= x"21";
             when x"116" => D <= x"C6";
             when x"117" => D <= x"02";
             when x"118" => D <= x"11";
             when x"119" => D <= x"F5";
             when x"11A" => D <= x"40";
             when x"11B" => D <= x"CD";
             when x"11C" => D <= x"0A";
             when x"11D" => D <= x"01";
             when x"11E" => D <= x"11";
             when x"11F" => D <= x"1D";
             when x"120" => D <= x"41";
             when x"121" => D <= x"3E";
             when x"122" => D <= x"0A";
             when x"123" => D <= x"21";
             when x"124" => D <= x"08";
             when x"125" => D <= x"03";
             when x"126" => D <= x"E5";
             when x"127" => D <= x"F5";
             when x"128" => D <= x"D5";
             when x"129" => D <= x"CD";
             when x"12A" => D <= x"0A";
             when x"12B" => D <= x"01";
             when x"12C" => D <= x"D1";
             when x"12D" => D <= x"F1";
             when x"12E" => D <= x"21";
             when x"12F" => D <= x"28";
             when x"130" => D <= x"00";
             when x"131" => D <= x"19";
             when x"132" => D <= x"54";
             when x"133" => D <= x"5D";
             when x"134" => D <= x"E1";
             when x"135" => D <= x"3D";
             when x"136" => D <= x"20";
             when x"137" => D <= x"EB";
             when x"138" => D <= x"21";
             when x"139" => D <= x"E7";
             when x"13A" => D <= x"02";
             when x"13B" => D <= x"CD";
             when x"13C" => D <= x"0A";
             when x"13D" => D <= x"01";
             when x"13E" => D <= x"C9";
             when x"13F" => D <= x"21";
             when x"140" => D <= x"1E";
             when x"141" => D <= x"41";
             when x"142" => D <= x"11";
             when x"143" => D <= x"00";
             when x"144" => D <= x"B0";
             when x"145" => D <= x"01";
             when x"146" => D <= x"1E";
             when x"147" => D <= x"00";
             when x"148" => D <= x"ED";
             when x"149" => D <= x"B0";
             when x"14A" => D <= x"21";
             when x"14B" => D <= x"46";
             when x"14C" => D <= x"41";
             when x"14D" => D <= x"11";
             when x"14E" => D <= x"1E";
             when x"14F" => D <= x"41";
             when x"150" => D <= x"06";
             when x"151" => D <= x"09";
             when x"152" => D <= x"C5";
             when x"153" => D <= x"E5";
             when x"154" => D <= x"D5";
             when x"155" => D <= x"01";
             when x"156" => D <= x"1E";
             when x"157" => D <= x"00";
             when x"158" => D <= x"ED";
             when x"159" => D <= x"B0";
             when x"15A" => D <= x"D1";
             when x"15B" => D <= x"21";
             when x"15C" => D <= x"28";
             when x"15D" => D <= x"00";
             when x"15E" => D <= x"19";
             when x"15F" => D <= x"54";
             when x"160" => D <= x"5D";
             when x"161" => D <= x"E1";
             when x"162" => D <= x"01";
             when x"163" => D <= x"28";
             when x"164" => D <= x"00";
             when x"165" => D <= x"09";
             when x"166" => D <= x"C1";
             when x"167" => D <= x"10";
             when x"168" => D <= x"E9";
             when x"169" => D <= x"21";
             when x"16A" => D <= x"00";
             when x"16B" => D <= x"B0";
             when x"16C" => D <= x"01";
             when x"16D" => D <= x"1E";
             when x"16E" => D <= x"00";
             when x"16F" => D <= x"ED";
             when x"170" => D <= x"B0";
             when x"171" => D <= x"C9";
             when x"172" => D <= x"CD";
             when x"173" => D <= x"AC";
             when x"174" => D <= x"01";
             when x"175" => D <= x"FE";
             when x"176" => D <= x"01";
             when x"177" => D <= x"20";
             when x"178" => D <= x"07";
             when x"179" => D <= x"CD";
             when x"17A" => D <= x"E7";
             when x"17B" => D <= x"01";
             when x"17C" => D <= x"3E";
             when x"17D" => D <= x"01";
             when x"17E" => D <= x"18";
             when x"17F" => D <= x"09";
             when x"180" => D <= x"FE";
             when x"181" => D <= x"02";
             when x"182" => D <= x"20";
             when x"183" => D <= x"10";
             when x"184" => D <= x"CD";
             when x"185" => D <= x"C3";
             when x"186" => D <= x"01";
             when x"187" => D <= x"3E";
             when x"188" => D <= x"80";
             when x"189" => D <= x"D3";
             when x"18A" => D <= x"01";
             when x"18B" => D <= x"11";
             when x"18C" => D <= x"E0";
             when x"18D" => D <= x"FF";
             when x"18E" => D <= x"21";
             when x"18F" => D <= x"00";
             when x"190" => D <= x"91";
             when x"191" => D <= x"CD";
             when x"192" => D <= x"0A";
             when x"193" => D <= x"01";
             when x"194" => D <= x"C9";
             when x"195" => D <= x"11";
             when x"196" => D <= x"00";
             when x"197" => D <= x"91";
             when x"198" => D <= x"21";
             when x"199" => D <= x"29";
             when x"19A" => D <= x"03";
             when x"19B" => D <= x"01";
             when x"19C" => D <= x"21";
             when x"19D" => D <= x"00";
             when x"19E" => D <= x"ED";
             when x"19F" => D <= x"B0";
             when x"1A0" => D <= x"11";
             when x"1A1" => D <= x"E0";
             when x"1A2" => D <= x"FF";
             when x"1A3" => D <= x"21";
             when x"1A4" => D <= x"00";
             when x"1A5" => D <= x"91";
             when x"1A6" => D <= x"01";
             when x"1A7" => D <= x"20";
             when x"1A8" => D <= x"00";
             when x"1A9" => D <= x"ED";
             when x"1AA" => D <= x"B0";
             when x"1AB" => D <= x"C9";
             when x"1AC" => D <= x"DB";
             when x"1AD" => D <= x"70";
             when x"1AE" => D <= x"C9";
             when x"1AF" => D <= x"3A";
             when x"1B0" => D <= x"00";
             when x"1B1" => D <= x"92";
             when x"1B2" => D <= x"3D";
             when x"1B3" => D <= x"32";
             when x"1B4" => D <= x"00";
             when x"1B5" => D <= x"92";
             when x"1B6" => D <= x"C9";
             when x"1B7" => D <= x"3A";
             when x"1B8" => D <= x"01";
             when x"1B9" => D <= x"92";
             when x"1BA" => D <= x"3D";
             when x"1BB" => D <= x"20";
             when x"1BC" => D <= x"02";
             when x"1BD" => D <= x"3E";
             when x"1BE" => D <= x"10";
             when x"1BF" => D <= x"32";
             when x"1C0" => D <= x"01";
             when x"1C1" => D <= x"92";
             when x"1C2" => D <= x"C9";
             when x"1C3" => D <= x"3A";
             when x"1C4" => D <= x"00";
             when x"1C5" => D <= x"91";
             when x"1C6" => D <= x"32";
             when x"1C7" => D <= x"02";
             when x"1C8" => D <= x"92";
             when x"1C9" => D <= x"3A";
             when x"1CA" => D <= x"10";
             when x"1CB" => D <= x"91";
             when x"1CC" => D <= x"32";
             when x"1CD" => D <= x"03";
             when x"1CE" => D <= x"92";
             when x"1CF" => D <= x"21";
             when x"1D0" => D <= x"01";
             when x"1D1" => D <= x"91";
             when x"1D2" => D <= x"11";
             when x"1D3" => D <= x"00";
             when x"1D4" => D <= x"91";
             when x"1D5" => D <= x"01";
             when x"1D6" => D <= x"1F";
             when x"1D7" => D <= x"00";
             when x"1D8" => D <= x"ED";
             when x"1D9" => D <= x"B0";
             when x"1DA" => D <= x"3A";
             when x"1DB" => D <= x"02";
             when x"1DC" => D <= x"92";
             when x"1DD" => D <= x"32";
             when x"1DE" => D <= x"0F";
             when x"1DF" => D <= x"91";
             when x"1E0" => D <= x"3A";
             when x"1E1" => D <= x"03";
             when x"1E2" => D <= x"92";
             when x"1E3" => D <= x"32";
             when x"1E4" => D <= x"1F";
             when x"1E5" => D <= x"91";
             when x"1E6" => D <= x"C9";
             when x"1E7" => D <= x"3A";
             when x"1E8" => D <= x"0F";
             when x"1E9" => D <= x"91";
             when x"1EA" => D <= x"32";
             when x"1EB" => D <= x"02";
             when x"1EC" => D <= x"92";
             when x"1ED" => D <= x"3A";
             when x"1EE" => D <= x"1F";
             when x"1EF" => D <= x"91";
             when x"1F0" => D <= x"32";
             when x"1F1" => D <= x"03";
             when x"1F2" => D <= x"92";
             when x"1F3" => D <= x"21";
             when x"1F4" => D <= x"1E";
             when x"1F5" => D <= x"91";
             when x"1F6" => D <= x"11";
             when x"1F7" => D <= x"1F";
             when x"1F8" => D <= x"91";
             when x"1F9" => D <= x"01";
             when x"1FA" => D <= x"1F";
             when x"1FB" => D <= x"00";
             when x"1FC" => D <= x"ED";
             when x"1FD" => D <= x"B8";
             when x"1FE" => D <= x"3A";
             when x"1FF" => D <= x"02";
             when x"200" => D <= x"92";
             when x"201" => D <= x"32";
             when x"202" => D <= x"00";
             when x"203" => D <= x"91";
             when x"204" => D <= x"3A";
             when x"205" => D <= x"03";
             when x"206" => D <= x"92";
             when x"207" => D <= x"32";
             when x"208" => D <= x"10";
             when x"209" => D <= x"91";
             when x"20A" => D <= x"C9";
             when x"20B" => D <= x"21";
             when x"20C" => D <= x"62";
             when x"20D" => D <= x"03";
             when x"20E" => D <= x"11";
             when x"20F" => D <= x"88";
             when x"210" => D <= x"4E";
             when x"211" => D <= x"01";
             when x"212" => D <= x"08";
             when x"213" => D <= x"00";
             when x"214" => D <= x"ED";
             when x"215" => D <= x"B0";
             when x"216" => D <= x"C9";
             when x"217" => D <= x"5A";
             when x"218" => D <= x"38";
             when x"219" => D <= x"30";
             when x"21A" => D <= x"20";
             when x"21B" => D <= x"53";
             when x"21C" => D <= x"59";
             when x"21D" => D <= x"53";
             when x"21E" => D <= x"54";
             when x"21F" => D <= x"45";
             when x"220" => D <= x"4D";
             when x"221" => D <= x"20";
             when x"222" => D <= x"4F";
             when x"223" => D <= x"4E";
             when x"224" => D <= x"20";
             when x"225" => D <= x"43";
             when x"226" => D <= x"48";
             when x"227" => D <= x"49";
             when x"228" => D <= x"50";
             when x"229" => D <= x"00";
             when x"22A" => D <= x"52";
             when x"22B" => D <= x"4F";
             when x"22C" => D <= x"4E";
             when x"22D" => D <= x"49";
             when x"22E" => D <= x"56";
             when x"22F" => D <= x"4F";
             when x"230" => D <= x"4E";
             when x"231" => D <= x"20";
             when x"232" => D <= x"43";
             when x"233" => D <= x"4F";
             when x"234" => D <= x"53";
             when x"235" => D <= x"54";
             when x"236" => D <= x"41";
             when x"237" => D <= x"20";
             when x"238" => D <= x"32";
             when x"239" => D <= x"30";
             when x"23A" => D <= x"30";
             when x"23B" => D <= x"38";
             when x"23C" => D <= x"00";
             when x"23D" => D <= x"20";
             when x"23E" => D <= x"20";
             when x"23F" => D <= x"7C";
             when x"240" => D <= x"21";
             when x"241" => D <= x"23";
             when x"242" => D <= x"24";
             when x"243" => D <= x"25";
             when x"244" => D <= x"26";
             when x"245" => D <= x"2F";
             when x"246" => D <= x"28";
             when x"247" => D <= x"29";
             when x"248" => D <= x"3D";
             when x"249" => D <= x"3F";
             when x"24A" => D <= x"2A";
             when x"24B" => D <= x"60";
             when x"24C" => D <= x"2B";
             when x"24D" => D <= x"B4";
             when x"24E" => D <= x"E7";
             when x"24F" => D <= x"7E";
             when x"250" => D <= x"5E";
             when x"251" => D <= x"2C";
             when x"252" => D <= x"2E";
             when x"253" => D <= x"3B";
             when x"254" => D <= x"3A";
             when x"255" => D <= x"5C";
             when x"256" => D <= x"3C";
             when x"257" => D <= x"3E";
             when x"258" => D <= x"00";
             when x"259" => D <= x"20";
             when x"25A" => D <= x"20";
             when x"25B" => D <= x"41";
             when x"25C" => D <= x"42";
             when x"25D" => D <= x"43";
             when x"25E" => D <= x"44";
             when x"25F" => D <= x"45";
             when x"260" => D <= x"46";
             when x"261" => D <= x"47";
             when x"262" => D <= x"48";
             when x"263" => D <= x"49";
             when x"264" => D <= x"4A";
             when x"265" => D <= x"4B";
             when x"266" => D <= x"4C";
             when x"267" => D <= x"4D";
             when x"268" => D <= x"4E";
             when x"269" => D <= x"4F";
             when x"26A" => D <= x"50";
             when x"26B" => D <= x"51";
             when x"26C" => D <= x"52";
             when x"26D" => D <= x"53";
             when x"26E" => D <= x"54";
             when x"26F" => D <= x"55";
             when x"270" => D <= x"56";
             when x"271" => D <= x"57";
             when x"272" => D <= x"58";
             when x"273" => D <= x"59";
             when x"274" => D <= x"5A";
             when x"275" => D <= x"00";
             when x"276" => D <= x"20";
             when x"277" => D <= x"20";
             when x"278" => D <= x"61";
             when x"279" => D <= x"62";
             when x"27A" => D <= x"63";
             when x"27B" => D <= x"64";
             when x"27C" => D <= x"65";
             when x"27D" => D <= x"66";
             when x"27E" => D <= x"67";
             when x"27F" => D <= x"68";
             when x"280" => D <= x"69";
             when x"281" => D <= x"6A";
             when x"282" => D <= x"6B";
             when x"283" => D <= x"6C";
             when x"284" => D <= x"6D";
             when x"285" => D <= x"6E";
             when x"286" => D <= x"6F";
             when x"287" => D <= x"70";
             when x"288" => D <= x"71";
             when x"289" => D <= x"72";
             when x"28A" => D <= x"73";
             when x"28B" => D <= x"74";
             when x"28C" => D <= x"75";
             when x"28D" => D <= x"76";
             when x"28E" => D <= x"77";
             when x"28F" => D <= x"78";
             when x"290" => D <= x"79";
             when x"291" => D <= x"7A";
             when x"292" => D <= x"00";
             when x"293" => D <= x"20";
             when x"294" => D <= x"20";
             when x"295" => D <= x"20";
             when x"296" => D <= x"20";
             when x"297" => D <= x"20";
             when x"298" => D <= x"20";
             when x"299" => D <= x"20";
             when x"29A" => D <= x"20";
             when x"29B" => D <= x"20";
             when x"29C" => D <= x"30";
             when x"29D" => D <= x"31";
             when x"29E" => D <= x"32";
             when x"29F" => D <= x"33";
             when x"2A0" => D <= x"34";
             when x"2A1" => D <= x"35";
             when x"2A2" => D <= x"36";
             when x"2A3" => D <= x"37";
             when x"2A4" => D <= x"38";
             when x"2A5" => D <= x"39";
             when x"2A6" => D <= x"00";
             when x"2A7" => D <= x"02";
             when x"2A8" => D <= x"03";
             when x"2A9" => D <= x"04";
             when x"2AA" => D <= x"0B";
             when x"2AB" => D <= x"0C";
             when x"2AC" => D <= x"0D";
             when x"2AD" => D <= x"0E";
             when x"2AE" => D <= x"12";
             when x"2AF" => D <= x"18";
             when x"2B0" => D <= x"19";
             when x"2B1" => D <= x"1A";
             when x"2B2" => D <= x"1B";
             when x"2B3" => D <= x"E8";
             when x"2B4" => D <= x"E9";
             when x"2B5" => D <= x"EB";
             when x"2B6" => D <= x"BB";
             when x"2B7" => D <= x"BC";
             when x"2B8" => D <= x"8A";
             when x"2B9" => D <= x"86";
             when x"2BA" => D <= x"87";
             when x"2BB" => D <= x"81";
             when x"2BC" => D <= x"80";
             when x"2BD" => D <= x"01";
             when x"2BE" => D <= x"06";
             when x"2BF" => D <= x"07";
             when x"2C0" => D <= x"08";
             when x"2C1" => D <= x"09";
             when x"2C2" => D <= x"0A";
             when x"2C3" => D <= x"1D";
             when x"2C4" => D <= x"1F";
             when x"2C5" => D <= x"00";
             when x"2C6" => D <= x"96";
             when x"2C7" => D <= x"9A";
             when x"2C8" => D <= x"9A";
             when x"2C9" => D <= x"9A";
             when x"2CA" => D <= x"9A";
             when x"2CB" => D <= x"9A";
             when x"2CC" => D <= x"9A";
             when x"2CD" => D <= x"9A";
             when x"2CE" => D <= x"9A";
             when x"2CF" => D <= x"9A";
             when x"2D0" => D <= x"9A";
             when x"2D1" => D <= x"9A";
             when x"2D2" => D <= x"9A";
             when x"2D3" => D <= x"9A";
             when x"2D4" => D <= x"9A";
             when x"2D5" => D <= x"9A";
             when x"2D6" => D <= x"9A";
             when x"2D7" => D <= x"9A";
             when x"2D8" => D <= x"9A";
             when x"2D9" => D <= x"9A";
             when x"2DA" => D <= x"9A";
             when x"2DB" => D <= x"9A";
             when x"2DC" => D <= x"9A";
             when x"2DD" => D <= x"9A";
             when x"2DE" => D <= x"9A";
             when x"2DF" => D <= x"9A";
             when x"2E0" => D <= x"9A";
             when x"2E1" => D <= x"9A";
             when x"2E2" => D <= x"9A";
             when x"2E3" => D <= x"9A";
             when x"2E4" => D <= x"9A";
             when x"2E5" => D <= x"9C";
             when x"2E6" => D <= x"00";
             when x"2E7" => D <= x"93";
             when x"2E8" => D <= x"9A";
             when x"2E9" => D <= x"9A";
             when x"2EA" => D <= x"9A";
             when x"2EB" => D <= x"9A";
             when x"2EC" => D <= x"9A";
             when x"2ED" => D <= x"9A";
             when x"2EE" => D <= x"9A";
             when x"2EF" => D <= x"9A";
             when x"2F0" => D <= x"9A";
             when x"2F1" => D <= x"9A";
             when x"2F2" => D <= x"9A";
             when x"2F3" => D <= x"9A";
             when x"2F4" => D <= x"9A";
             when x"2F5" => D <= x"9A";
             when x"2F6" => D <= x"9A";
             when x"2F7" => D <= x"9A";
             when x"2F8" => D <= x"9A";
             when x"2F9" => D <= x"9A";
             when x"2FA" => D <= x"9A";
             when x"2FB" => D <= x"9A";
             when x"2FC" => D <= x"9A";
             when x"2FD" => D <= x"9A";
             when x"2FE" => D <= x"9A";
             when x"2FF" => D <= x"9A";
             when x"300" => D <= x"9A";
             when x"301" => D <= x"9A";
             when x"302" => D <= x"9A";
             when x"303" => D <= x"9A";
             when x"304" => D <= x"9A";
             when x"305" => D <= x"9A";
             when x"306" => D <= x"99";
             when x"307" => D <= x"00";
             when x"308" => D <= x"95";
             when x"309" => D <= x"20";
             when x"30A" => D <= x"20";
             when x"30B" => D <= x"20";
             when x"30C" => D <= x"20";
             when x"30D" => D <= x"20";
             when x"30E" => D <= x"20";
             when x"30F" => D <= x"20";
             when x"310" => D <= x"20";
             when x"311" => D <= x"20";
             when x"312" => D <= x"20";
             when x"313" => D <= x"20";
             when x"314" => D <= x"20";
             when x"315" => D <= x"20";
             when x"316" => D <= x"20";
             when x"317" => D <= x"20";
             when x"318" => D <= x"20";
             when x"319" => D <= x"20";
             when x"31A" => D <= x"20";
             when x"31B" => D <= x"20";
             when x"31C" => D <= x"20";
             when x"31D" => D <= x"20";
             when x"31E" => D <= x"20";
             when x"31F" => D <= x"20";
             when x"320" => D <= x"20";
             when x"321" => D <= x"20";
             when x"322" => D <= x"20";
             when x"323" => D <= x"20";
             when x"324" => D <= x"20";
             when x"325" => D <= x"20";
             when x"326" => D <= x"20";
             when x"327" => D <= x"95";
             when x"328" => D <= x"00";
             when x"329" => D <= x"20";
             when x"32A" => D <= x"20";
             when x"32B" => D <= x"20";
             when x"32C" => D <= x"5A";
             when x"32D" => D <= x"38";
             when x"32E" => D <= x"30";
             when x"32F" => D <= x"20";
             when x"330" => D <= x"53";
             when x"331" => D <= x"59";
             when x"332" => D <= x"53";
             when x"333" => D <= x"54";
             when x"334" => D <= x"45";
             when x"335" => D <= x"4D";
             when x"336" => D <= x"20";
             when x"337" => D <= x"20";
             when x"338" => D <= x"20";
             when x"339" => D <= x"20";
             when x"33A" => D <= x"52";
             when x"33B" => D <= x"4F";
             when x"33C" => D <= x"4E";
             when x"33D" => D <= x"49";
             when x"33E" => D <= x"56";
             when x"33F" => D <= x"4F";
             when x"340" => D <= x"4E";
             when x"341" => D <= x"20";
             when x"342" => D <= x"20";
             when x"343" => D <= x"43";
             when x"344" => D <= x"4F";
             when x"345" => D <= x"53";
             when x"346" => D <= x"54";
             when x"347" => D <= x"41";
             when x"348" => D <= x"20";
             when x"349" => D <= x"00";
             when x"34A" => D <= x"44";
             when x"34B" => D <= x"45";
             when x"34C" => D <= x"31";
             when x"34D" => D <= x"20";
             when x"34E" => D <= x"56";
             when x"34F" => D <= x"65";
             when x"350" => D <= x"72";
             when x"351" => D <= x"73";
             when x"352" => D <= x"69";
             when x"353" => D <= x"6F";
             when x"354" => D <= x"6E";
             when x"355" => D <= x"00";
             when x"356" => D <= x"53";
             when x"357" => D <= x"33";
             when x"358" => D <= x"45";
             when x"359" => D <= x"20";
             when x"35A" => D <= x"56";
             when x"35B" => D <= x"65";
             when x"35C" => D <= x"72";
             when x"35D" => D <= x"73";
             when x"35E" => D <= x"69";
             when x"35F" => D <= x"6F";
             when x"360" => D <= x"6E";
             when x"361" => D <= x"00";
             when x"362" => D <= x"FF";
             when x"363" => D <= x"81";
             when x"364" => D <= x"99";
             when x"365" => D <= x"A5";
             when x"366" => D <= x"BD";
             when x"367" => D <= x"A5";
             when x"368" => D <= x"81";
             when x"369" => D <= x"FF";
             when others => D <="ZZZZZZZZ";
        end case;
 end if;
end process;
end;