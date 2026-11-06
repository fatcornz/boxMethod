--------------------------------------------------------------------------------
-- top.vhd -- Digital-only, simulation-first OPEN-LOOP LLC controller
-- Target: Arty S7, 100 MHz FPGA clock.
-- NO INA229/SPI, no voltage acquisition, and NO 50 V regulation implemented.
-- The future voltage feedback interface is reserved internally and tied off.
-- These are LOGIC COMMANDS, not GaN gate-drive signals. Separate isolation,
-- HOT-side hardware fault shutdown, and safe power-up interlocking REQUIRED.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity top is
    generic (
        DEADTIME_CLKS : positive := 1
    );
    port (
        clk_100mhz     : in  std_logic;
        reset          : in  std_logic; -- active HIGH
        enable_request : in  std_logic; -- active HIGH
        frequency_sel  : in  std_logic_vector(1 downto 0);
        fault_in       : in  std_logic; -- active HIGH, externally conditioned
        clear_fault    : in  std_logic; -- only permitted while disabled
        gate_high_cmd  : out std_logic;
        gate_low_cmd   : out std_logic;
        fault_latched  : out std_logic
    );
end entity top;

architecture rtl of top is
    signal half_period_clks : unsigned(15 downto 0);
    signal phase_cmd        : std_logic;
    signal high_internal    : std_logic;
    signal low_internal     : std_logic;
    signal inhibit         : std_logic;
    signal run_allowed     : std_logic;
    signal feedback_vbus_code  : unsigned(19 downto 0);
    signal feedback_vbus_valid : std_logic;
begin
    -- FUTURE SENSOR HOOK: replace these constant assignments with outputs
    -- of ina229_interface.vhd when isolated SPI measurements are implemented.
    feedback_vbus_code  <= (others => '0');
    feedback_vbus_valid <= '0';

    -- An external hardware fault must inhibit the actual driver independently.
    run_allowed <= enable_request and not reset and not inhibit;

    u_controller : entity work.controller_stub
        port map (
            clk              => clk_100mhz,
            reset            => reset,
            frequency_sel    => frequency_sel,
            vbus_code        => feedback_vbus_code,
            vbus_valid       => feedback_vbus_valid,
            half_period_clks => half_period_clks
        );

    u_frequency : entity work.frequency_generator
        port map (
            clk              => clk_100mhz,
            reset            => reset,
            enable           => run_allowed,
            half_period_clks => half_period_clks,
            phase_cmd        => phase_cmd
        );

    u_deadtime : entity work.deadtime_generator
        generic map (DEADTIME_CLKS => DEADTIME_CLKS)
        port map (
            clk       => clk_100mhz,
            reset     => reset,
            enable    => run_allowed,
            phase_cmd => phase_cmd,
            gate_high => high_internal,
            gate_low  => low_internal
        );

    u_fault : entity work.fault_controller
        port map (
            clk           => clk_100mhz,
            reset         => reset,
            run_enable    => enable_request,
            fault_in      => fault_in,
            clear_fault   => clear_fault,
            fault_latched => fault_latched,
            inhibit       => inhibit
        );

    -- Immediate combinational output masking on fault, reset, or disable.
    -- This is NOT a substitute for an independent hardware shutdown path.
    gate_high_cmd <= high_internal and run_allowed;
    gate_low_cmd  <= low_internal and run_allowed;
end architecture rtl;
