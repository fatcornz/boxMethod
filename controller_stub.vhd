--------------------------------------------------------------------------------
-- controller_stub.vhd
-- Manual example-frequency selector; no voltage measurement or PI logic yet.
-- Reserved future input: vbus_code = INA229 VBUS bits [23:4] (20 bits).
-- vbus_valid is a one-clock pulse whenever a fresh reading is available.
-- 50.0 V corresponds to VBUS code 256000 (195.3125 uV/LSB).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity controller_stub is
    port (
        clk              : in  std_logic;
        reset            : in  std_logic;
        frequency_sel    : in  std_logic_vector(1 downto 0);
        vbus_code        : in  unsigned(19 downto 0); -- reserved, ignored
        vbus_valid       : in  std_logic;             -- reserved, ignored
        half_period_clks : out unsigned(15 downto 0)
    );
end entity controller_stub;

architecture rtl of controller_stub is
    signal selected_half : unsigned(15 downto 0) := to_unsigned(500, 16);
begin
    half_period_clks <= selected_half;
    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                selected_half <= to_unsigned(500, 16);
            else
                case frequency_sel is
                    when "00"   => selected_half <= to_unsigned(500, 16); -- 100 kHz
                    when "01"   => selected_half <= to_unsigned(250, 16); -- 200 kHz
                    when "10"   => selected_half <= to_unsigned(200, 16); -- 250 kHz
                    when others => selected_half <= to_unsigned(100, 16); -- 500 kHz
                end case;
            end if;
        end if;
    end process;
end architecture rtl;
