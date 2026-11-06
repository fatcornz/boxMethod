-- MSB-first SPI mode 1 (idle low, launch rising, sample falling).
-- tx_data is left aligned; rx_data contains right-aligned received bits.
-- start is accepted only while busy=0; done pulses for one clk.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_master is
    generic (HALF_PERIOD_CLKS : positive := 50);
    port (
        clk, reset, start : in std_logic;
        bit_count : in natural range 1 to 32;
        tx_data : in std_logic_vector(31 downto 0);
        rx_data : out std_logic_vector(31 downto 0);
        busy, done : out std_logic;
        spi_cs_n, spi_sclk, spi_mosi : out std_logic;
        spi_miso : in std_logic
    );
end entity;

architecture rtl of spi_master is
    type state_t is (IDLE, SETUP, HIGH_PHASE, LOW_PHASE, HOLD_CS, GAP);
    signal state : state_t := IDLE;
    signal ticks : natural range 0 to HALF_PERIOD_CLKS-1 := 0;
    signal remaining : natural range 1 to 32 := 1;
    signal tx, rx : std_logic_vector(31 downto 0) := (others => '0');
    signal miso_meta, miso_sync : std_logic := '0';
    attribute ASYNC_REG : string;
    attribute ASYNC_REG of miso_meta, miso_sync : signal is "TRUE";
begin
    -- Allow synchronizer latency before sampling the returned MISO signal.
    -- Board/isolation propagation delay must ALSO fit within the half period.
    assert HALF_PERIOD_CLKS >= 5 severity failure;
    busy <= '0' when state = IDLE else '1';
    process(clk)
    begin
        if rising_edge(clk) then
            miso_meta <= spi_miso;
            miso_sync <= miso_meta;
            done <= '0';
            if reset = '1' then
                state <= IDLE;
                ticks <= 0;
                remaining <= 1;
                tx <= (others => '0');
                rx <= (others => '0');
                rx_data <= (others => '0');
                miso_meta <= '0';
                miso_sync <= '0';
                spi_cs_n <= '1';
                spi_sclk <= '0';
                spi_mosi <= '0';
            elsif state = IDLE then
                if start = '1' then
                    tx <= tx_data;
                    rx <= (others => '0');
                    remaining <= bit_count;
                    spi_cs_n <= '0';
                    ticks <= HALF_PERIOD_CLKS-1;
                    state <= SETUP;
                end if;
            elsif ticks /= 0 then
                ticks <= ticks - 1;
            else
                ticks <= HALF_PERIOD_CLKS-1;
                case state is
                    when SETUP | LOW_PHASE =>
                        spi_sclk <= '1';
                        spi_mosi <= tx(31);
                        tx <= tx(30 downto 0) & '0';
                        state <= HIGH_PHASE;
                    when HIGH_PHASE =>
                        spi_sclk <= '0';
                        rx <= rx(30 downto 0) & miso_sync;
                        if remaining = 1 then
                            state <= HOLD_CS;
                        else
                            remaining <= remaining - 1;
                            state <= LOW_PHASE;
                        end if;
                    when HOLD_CS =>
                        spi_cs_n <= '1';
                        spi_mosi <= '0';
                        rx_data <= rx;
                        state <= GAP;
                    when GAP =>
                        done <= '1';
                        state <= IDLE;
                    when others => null;
                end case;
            end if;
        end if;
    end process;
end architecture;
