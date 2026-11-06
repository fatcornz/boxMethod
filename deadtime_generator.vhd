--------------------------------------------------------------------------------
-- deadtime_generator.vhd
-- Two non-overlapping, active-high HALF-BRIDGE logic commands.
-- On each phase transition, both outputs are forced off for DEADTIME_CLKS
-- FPGA clock cycles before the newly requested output may turn on.
-- NOTE: FPGA dead time is NOT independent protection hardware.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity deadtime_generator is
    generic (
        DEADTIME_CLKS : positive := 5
    );
    port (
        clk       : in  std_logic;
        reset     : in  std_logic;
        enable    : in  std_logic;
        phase_cmd : in  std_logic;
        gate_high : out std_logic;
        gate_low  : out std_logic
    );
end entity deadtime_generator;

architecture rtl of deadtime_generator is
    type state_t is (DISABLED, BLANKING, DRIVING);
    signal state           : state_t := DISABLED;
    signal pending_phase   : std_logic := '0';
    signal blank_remaining : natural range 0 to DEADTIME_CLKS - 1 := 0;
    signal high_reg        : std_logic := '0';
    signal low_reg         : std_logic := '0';
begin
    gate_high <= high_reg;
    gate_low  <= low_reg;

    process (clk)
    begin
        if rising_edge(clk) then
            if reset = '1' or enable = '0' then
                state           <= DISABLED;
                pending_phase   <= '0';
                blank_remaining <= 0;
                high_reg        <= '0';
                low_reg         <= '0';
            else
                case state is
                    when DISABLED =>
                        pending_phase   <= phase_cmd;
                        blank_remaining <= DEADTIME_CLKS - 1;
                        high_reg        <= '0';
                        low_reg         <= '0';
                        state           <= BLANKING;

                    when BLANKING =>
                        high_reg <= '0';
                        low_reg  <= '0';
                        if phase_cmd /= pending_phase then
                            pending_phase   <= phase_cmd;
                            blank_remaining <= DEADTIME_CLKS - 1;
                        elsif blank_remaining = 0 then
                            high_reg <= pending_phase;
                            low_reg  <= not pending_phase;
                            state    <= DRIVING;
                        else
                            blank_remaining <= blank_remaining - 1;
                        end if;

                    when DRIVING =>
                        if phase_cmd /= pending_phase then
                            pending_phase   <= phase_cmd;
                            blank_remaining <= DEADTIME_CLKS - 1;
                            high_reg        <= '0';
                            low_reg         <= '0';
                            state           <= BLANKING;
                        end if;
                end case;
            end if;
        end if;
    end process;
end architecture rtl;
