-- Voltage-only INA229 acquisition, synchronous active-high reset.
-- Reset after the sensor supply is stable. Faults latch until reset.
-- vbus_valid is a one-clock NEW SAMPLE pulse, not a persistent health flag.
-- No current calibration, regulation, or gate-drive protection implemented.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ina229_interface is
    generic (
        SPI_HALF_PERIOD_CLKS : positive := 50; -- 1 MHz at 100 MHz clk
        POWERUP_CLKS : positive := 100000;     -- 1 ms at 100 MHz
        SAMPLE_GAP_CLKS : positive := 100000;
        MAX_READY_POLLS : positive := 32
    );
    port (
        clk, reset : in std_logic;
        spi_miso : in std_logic;
        spi_cs_n, spi_sclk, spi_mosi : out std_logic;
        vbus_code : out unsigned(19 downto 0);
        vbus_valid : out std_logic;
        sensor_ready, sensor_fault : out std_logic
    );
end entity;

architecture rtl of ina229_interface is
    type state_t is (POWERUP, ISSUE, WAIT_SPI, SAMPLE_GAP, FAILED);
    type operation_t is (MANUFACTURER, DEVICE, CONFIG_WRITE, CONFIG_CHECK,
                         TRIGGER_SAMPLE, ADC_CHECK, READY_CHECK, BUS_READ);
    signal state : state_t := POWERUP;
    signal op : operation_t := MANUFACTURER;
    signal power_count : natural range 0 to POWERUP_CLKS-1 := 0;
    signal gap_count : natural range 0 to SAMPLE_GAP_CLKS-1 := 0;
    signal poll_count : natural range 0 to MAX_READY_POLLS-1 := 0;
    signal start, done, busy : std_logic := '0';
    signal tx, rx : std_logic_vector(31 downto 0);
    signal bits : natural range 1 to 32;
begin
    u_spi : entity work.spi_master
        generic map (HALF_PERIOD_CLKS => SPI_HALF_PERIOD_CLKS)
        port map (clk => clk, reset => reset, start => start,
                  bit_count => bits, tx_data => tx, rx_data => rx,
                  busy => busy, done => done, spi_cs_n => spi_cs_n,
                  spi_sclk => spi_sclk, spi_mosi => spi_mosi, spi_miso => spi_miso);

    -- Command byte = address[5:0] & '0' & read_not_write.
    -- 16-bit transfers are left aligned in the 32-bit transport word.
    process(op)
    begin
        bits <= 24;
        case op is
            when MANUFACTURER   => tx <= x"F9000000";
            when DEVICE         => tx <= x"FD000000";
            when CONFIG_WRITE   => tx <= x"00001000"; -- ADCRANGE=1
            when CONFIG_CHECK   => tx <= x"01000000";
            when TRIGGER_SAMPLE => tx <= x"04100000"; -- bus-only single shot, 50 us
            when ADC_CHECK      => tx <= x"05000000";
            when READY_CHECK    => tx <= x"2D000000";
            when BUS_READ       => tx <= x"15000000"; bits <= 32;
        end case;
    end process;

    process(clk)
    begin
        if rising_edge(clk) then
            start <= '0';
            vbus_valid <= '0';
            if reset = '1' then
                state <= POWERUP;
                op <= MANUFACTURER;
                power_count <= 0;
                gap_count <= 0;
                poll_count <= 0;
                vbus_code <= (others => '0');
                sensor_ready <= '0';
                sensor_fault <= '0';
            else
                case state is
                    when POWERUP =>
                        if power_count = POWERUP_CLKS-1 then
                            state <= ISSUE;
                        else
                            power_count <= power_count + 1;
                        end if;
                    when ISSUE =>
                        if busy = '0' then
                            start <= '1';
                            state <= WAIT_SPI;
                        end if;
                    when WAIT_SPI =>
                        if done = '1' then
                            state <= ISSUE;
                            case op is
                                when MANUFACTURER =>
                                    if rx(15 downto 0) = x"5449" then op <= DEVICE;
                                    else state <= FAILED; end if;
                                when DEVICE =>
                                    if rx(15 downto 4) = x"229" then op <= CONFIG_WRITE;
                                    else state <= FAILED; end if;
                                when CONFIG_WRITE => op <= CONFIG_CHECK;
                                when CONFIG_CHECK =>
                                    if rx(15 downto 0) = x"0010" then op <= TRIGGER_SAMPLE;
                                    else state <= FAILED; end if;
                                when TRIGGER_SAMPLE =>
                                    poll_count <= 0;
                                    op <= ADC_CHECK;
                                when ADC_CHECK =>
                                    if rx(15 downto 0) = x"1000" then op <= READY_CHECK;
                                    else state <= FAILED; end if;
                                when READY_CHECK =>
                                    if rx(0) /= '1' or rx(8) /= '0' then
                                        state <= FAILED; -- trim error / invalid response
                                    elsif rx(1) = '1' then
                                        op <= BUS_READ;
                                    elsif poll_count = MAX_READY_POLLS-1 then
                                        state <= FAILED;
                                    else
                                        poll_count <= poll_count + 1;
                                    end if;
                                when BUS_READ =>
                                    if rx(3 downto 0) /= x"0" or
                                       unsigned(rx(23 downto 4)) > to_unsigned(435200, 20) then
                                        state <= FAILED; -- invalid encoding / above 85 V
                                    else
                                        vbus_code <= unsigned(rx(23 downto 4));
                                        vbus_valid <= '1';
                                        sensor_ready <= '1';
                                        gap_count <= 0;
                                        state <= SAMPLE_GAP;
                                    end if;
                            end case;
                        end if;
                    when SAMPLE_GAP =>
                        if gap_count = SAMPLE_GAP_CLKS-1 then
                            op <= TRIGGER_SAMPLE;
                            state <= ISSUE;
                        else gap_count <= gap_count + 1;
                        end if;
                    when FAILED =>
                        sensor_ready <= '0';
                        sensor_fault <= '1';
                end case;
            end if;
        end if;
    end process;
end architecture;
