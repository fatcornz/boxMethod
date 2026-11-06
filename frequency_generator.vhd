--------------------------------------------------------------------------------
-- frequency_generator.vhd
-- Variable-frequency, nominally 50%-duty PHASE COMMAND, not a power-gate drive.
-- Uses clock enables/counters; no derived clock. New frequency settings take
-- effect at the low-to-high phase boundary, never mid-half-cycle.
-- f_switch ~= CLOCK_HZ / (2 * half_period_clks)
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity frequency_generator is
    port (
        clk              : in  std_logic;
        reset            : in  std_logic;
        enable           : in  std_logic;
        half_period_clks : in  unsigned(15 downto 0);
        phase_cmd        : out std_logic
    );
end entity frequency_generator;

architecture rtl of frequency_generator is
    signal count       : unsigned(15 downto 0) := (others => '0');
    signal active_half : unsigned(15 downto 0) := to_unsigned(200, 16);
    signal phase_reg   : std_logic := '0';

    function clamp_half(n : unsigned(15 downto 0)) return unsigned is
    begin
        if n < to_unsigned(2, 16) then
            return to_unsigned(2, 16);
        end if;
        return n;
    end function;
begin
    phase_cmd <= phase_reg;

    process (clk)
    begin
        if rising_edge(clk) then
            if reset = '1' or enable = '0' then
                count       <= (others => '0');
                active_half <= clamp_half(half_period_clks);
                phase_reg   <= '0';
            elsif count = active_half - 1 then
                count     <= (others => '0');
                phase_reg <= not phase_reg;
                if phase_reg = '0' then
                    -- Load a new value at the start of the high half-cycle.
                    active_half <= clamp_half(half_period_clks);
                end if;
            else
                count <= count + 1;
            end if;
        end if;
    end process;
end architecture rtl;
