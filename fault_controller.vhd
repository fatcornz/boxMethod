--------------------------------------------------------------------------------
-- fault_controller.vhd
-- Latches a (hardware-conditioned) digital fault. Clearing is only permitted
-- when run_enable is low and fault_in has returned low.
-- Fault input is also combined into the inhibit output without clock delay.
-- CRITICAL: An external comparator/fault latch MUST independently turn off
-- the GaN driver. This FPGA module is NOT an independent safety interlock.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity fault_controller is
    port (
        clk           : in  std_logic;
        reset         : in  std_logic;
        run_enable    : in  std_logic;
        fault_in      : in  std_logic;
        clear_fault   : in  std_logic;
        fault_latched : out std_logic;
        inhibit       : out std_logic
    );
end entity fault_controller;

architecture rtl of fault_controller is
    signal fault_meta : std_logic := '0';
    signal fault_sync : std_logic := '0';
    signal fault_reg  : std_logic := '0';
begin
    fault_latched <= fault_reg;
    inhibit       <= fault_in or fault_reg;

    process (clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                fault_meta <= '0';
                fault_sync <= '0';
                fault_reg  <= '0';
            else
                fault_meta <= fault_in;
                fault_sync <= fault_meta;
                if fault_sync = '1' then
                    fault_reg <= '1';
                elsif clear_fault = '1' and run_enable = '0' and fault_in = '0' then
                    fault_reg <= '0';
                end if;
            end if;
        end if;
    end process;
end architecture rtl;
