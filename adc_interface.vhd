--------------------------------------------------------------------------------
-- adc_interface.vhd
-- SPI master for the TI INA229 power monitor (through the isolation barrier).
-- Provides output voltage + current samples to the controller (feedback) and
-- non-latched OV / OC / sensor-health flags to fault_controller (fault sensing).
--
--  * SPI mode 1 (CPOL=0, CPHA=1), MSB first. Command byte = addr[5:0] & '0' & R/W
--    (R/W = '1' read). INA229 SCLK max 10 MHz.
--  * Samples are taken on a FIXED period (POLL_PERIOD_CLKS) so the PI loop's
--    sample time Ts is constant. POLL_PERIOD must be >= the INA229 conversion
--    cycle set by ADC_CONFIG_VALUE, or samples will repeat.
--  * vbus_code   : INA229 VBUS[23:4],   unsigned, 195.3125 uV/LSB (50.0 V = 256000)
--  * ishunt_code : INA229 VSHUNT[23:4], signed, 312.5 nV/LSB (ADCRANGE='0')
--                  or 78.125 nV/LSB (ADCRANGE='1');  I = code * LSB / R_SHUNT.
--                  VSHUNT is used instead of CURRENT so no SHUNT_CAL is needed
--                  (an unset SHUNT_CAL would read 0 A and silently disable OC).
--  * Sensor health: ID check at init, ADC_CONFIG write is read back, and the
--    readback is repeated every HEALTH_CHECK_POLLS samples. A dead or unpowered
--    isolated side reads all-0s/all-1s and is caught BEFORE that sample is
--    published, so the controller never regulates on a bogus 0 V reading.
--
-- Integration (top.vhd):
--   feedback_vbus_code / feedback_vbus_valid <= vbus_code / vbus_valid
--   fault_any   <= fault_in or meas_fault;   -- feed fault_any to u_fault.fault_in
--   run_allowed <= enable_request and not reset and not inhibit and sensor_ready;
--
-- CRITICAL: INA229 conversions take tens to hundreds of microseconds. This is
-- SUPERVISORY protection (output OV, sustained overload, lost feedback) only.
-- Shoot-through / short-circuit protection MUST be an independent hardware
-- comparator path that turns off the GaN driver directly.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity adc_interface is
    generic (
        SCLK_HALF_CLKS     : positive := 10;        -- 100 MHz / 20 = 5 MHz SCLK
        CS_GAP_CLKS        : positive := 20;        -- CS high time between frames
        STARTUP_CLKS       : positive := 100_000;   -- 1 ms after reset before first access
        RETRY_CLKS         : positive := 1_000_000; -- 10 ms between re-init attempts
        POLL_PERIOD_CLKS   : positive := 20_000;    -- 200 us -> 5 kS/s
        HEALTH_CHECK_POLLS : positive := 1;         -- verify sensor every N samples
        -- MODE=Bh (continuous VSHUNT+VBUS), VBUSCT=84 us, VSHCT=84 us, AVG=1
        -- -> ~168 us conversion cycle, fits inside POLL_PERIOD_CLKS.
        ADC_CONFIG_VALUE   : std_logic_vector(15 downto 0) := x"B240";
        ADCRANGE           : std_logic := '0';      -- '0' = +/-163.84 mV shunt range
        EXPECTED_DIE_ID    : std_logic_vector(11 downto 0) := x"229";
        OV_LIMIT_CODE      : natural := 281_600;    -- 55.0 V
        OC_LIMIT_CODE      : natural := 64_000;     -- 20 mV: PLACEHOLDER (10 A on 2 mOhm)
        FAULT_FILTER       : positive := 1          -- consecutive samples to assert OV/OC
    );
    port (
        clk           : in  std_logic;
        reset         : in  std_logic; -- active HIGH
        -- SPI to INA229
        spi_cs_n      : out std_logic;
        spi_sclk      : out std_logic;
        spi_mosi      : out std_logic;
        spi_miso      : in  std_logic;
        -- Feedback
        vbus_code     : out unsigned(19 downto 0);
        vbus_valid    : out std_logic; -- one-clock pulse per new, verified sample
        ishunt_code   : out signed(19 downto 0);
        ishunt_valid  : out std_logic;
        -- Fault sensing (NOT latched; fault_controller does the latching)
        sensor_ready  : out std_logic; -- configured and passing health checks
        sensor_fault  : out std_logic; -- ID/config/data check failed; clears on re-init
        ov_fault      : out std_logic;
        oc_fault      : out std_logic;
        meas_fault    : out std_logic; -- sensor_fault or ov_fault or oc_fault
        spi_err_count : out unsigned(7 downto 0) -- saturating, for debug/telemetry
    );
end entity adc_interface;

architecture rtl of adc_interface is
    function max_nat(a, b : natural) return natural is
    begin
        if a > b then
            return a;
        end if;
        return b;
    end function;

    -- INA229 register addresses (subset)
    constant REG_CONFIG     : natural := 16#00#;
    constant REG_ADC_CONFIG : natural := 16#01#;
    constant REG_VSHUNT     : natural := 16#04#;
    constant REG_VBUS       : natural := 16#05#;
    constant REG_MFG_ID     : natural := 16#3E#;
    constant REG_DEVICE_ID  : natural := 16#3F#;
    constant MFG_ID_TI      : std_logic_vector(15 downto 0) := x"5449"; -- "TI"

    -- CONFIG: RST=0, RSTACC=0, CONVDLY=0, TEMPCOMP=0, ADCRANGE, reserved
    constant CONFIG_VALUE : std_logic_vector(15 downto 0) :=
        "00000000000" & ADCRANGE & "0000";

    subtype byte_t is std_logic_vector(7 downto 0);

    function rd_cmd(addr : natural) return byte_t is
    begin
        return std_logic_vector(to_unsigned(addr, 6)) & "01";
    end function;

    function wr_cmd(addr : natural) return byte_t is
    begin
        return std_logic_vector(to_unsigned(addr, 6)) & "00";
    end function;

    ---------------------------------------------------------------- SPI engine
    constant SPI_TIMER_MAX : natural := max_nat(SCLK_HALF_CLKS, CS_GAP_CLKS) - 1;
    type spi_state_t is (SPI_IDLE, SPI_SETUP, SPI_HIGH, SPI_LOW, SPI_HOLD, SPI_GAP);
    signal spi_state : spi_state_t := SPI_IDLE;
    signal spi_timer : natural range 0 to SPI_TIMER_MAX := 0;
    signal spi_bits  : natural range 0 to 32 := 0;
    signal tx_shift  : std_logic_vector(31 downto 0) := (others => '0');
    signal rx_shift  : std_logic_vector(31 downto 0) := (others => '0');
    signal miso_meta : std_logic := '0';
    signal miso_sync : std_logic := '0';
    signal cs_n_reg  : std_logic := '1';
    signal sclk_reg  : std_logic := '0';
    signal mosi_reg  : std_logic := '0';

    -- sequencer -> engine (frame is left-aligned in spi_tx)
    signal spi_start : std_logic := '0';
    signal spi_nbits : natural range 0 to 32 := 32;
    signal spi_tx    : std_logic_vector(31 downto 0) := (others => '0');
    signal spi_done  : std_logic := '0';

    ---------------------------------------------------------------- sequencer
    type seq_state_t is (
        ST_WAIT, ST_RD_MFG, ST_RD_DEVID, ST_WR_CONFIG, ST_WR_ADCCFG, ST_VERIFY,
        ST_IDLE, ST_RD_VBUS, ST_RD_VSHUNT, ST_HEALTH, ST_PUBLISH, ST_FAIL
    );
    signal seq_state    : seq_state_t := ST_WAIT;
    signal issued       : std_logic := '0';
    signal wait_cnt     : natural range 0 to max_nat(STARTUP_CLKS, RETRY_CLKS) - 1
                          := STARTUP_CLKS - 1;
    signal poll_cnt     : natural range 0 to POLL_PERIOD_CLKS - 1 := 0;
    signal poll_pending : std_logic := '0';
    signal health_cnt   : natural range 0 to HEALTH_CHECK_POLLS - 1 := 0;
    signal vbus_raw     : std_logic_vector(19 downto 0) := (others => '0');
    signal vshunt_raw   : std_logic_vector(19 downto 0) := (others => '0');

    signal vbus_code_r   : unsigned(19 downto 0) := (others => '0');
    signal ishunt_code_r : signed(19 downto 0) := (others => '0');
    signal valid_r       : std_logic := '0';
    signal ready_r       : std_logic := '0';
    signal sensor_flt_r  : std_logic := '0';
    signal ov_r          : std_logic := '0';
    signal oc_r          : std_logic := '0';
    signal ov_cnt        : natural range 0 to FAULT_FILTER := 0;
    signal oc_cnt        : natural range 0 to FAULT_FILTER := 0;
    signal err_cnt       : unsigned(7 downto 0) := (others => '0');
begin
    spi_cs_n      <= cs_n_reg;
    spi_sclk      <= sclk_reg;
    spi_mosi      <= mosi_reg;
    vbus_code     <= vbus_code_r;
    vbus_valid    <= valid_r;
    ishunt_code   <= ishunt_code_r;
    ishunt_valid  <= valid_r;
    sensor_ready  <= ready_r;
    sensor_fault  <= sensor_flt_r;
    ov_fault      <= ov_r;
    oc_fault      <= oc_r;
    meas_fault    <= sensor_flt_r or ov_r or oc_r;
    spi_err_count <= err_cnt;

    ----------------------------------------------------------------------------
    -- SPI mode 1 bit engine. MOSI changes with the rising SCLK edge; MISO is
    -- sampled at the falling edge. Budget: INA229 t_DO + isolator round-trip
    -- delay must be < (SCLK_HALF_CLKS - 2) clocks (80 ns at the defaults).
    ----------------------------------------------------------------------------
    spi_engine : process (clk)
    begin
        if rising_edge(clk) then
            spi_done  <= '0';
            miso_meta <= spi_miso;
            miso_sync <= miso_meta;

            if reset = '1' then
                spi_state <= SPI_IDLE;
                spi_timer <= 0;
                spi_bits  <= 0;
                cs_n_reg  <= '1';
                sclk_reg  <= '0';
                mosi_reg  <= '0';
            else
                case spi_state is
                    when SPI_IDLE =>
                        cs_n_reg <= '1';
                        sclk_reg <= '0';
                        if spi_start = '1' then
                            tx_shift  <= spi_tx;
                            spi_bits  <= spi_nbits;
                            cs_n_reg  <= '0';
                            spi_timer <= SCLK_HALF_CLKS - 1;
                            spi_state <= SPI_SETUP;
                        end if;

                    when SPI_SETUP => -- CS setup time before the first edge
                        if spi_timer = 0 then
                            sclk_reg  <= '1';
                            mosi_reg  <= tx_shift(31);
                            tx_shift  <= tx_shift(30 downto 0) & '0';
                            spi_timer <= SCLK_HALF_CLKS - 1;
                            spi_state <= SPI_HIGH;
                        else
                            spi_timer <= spi_timer - 1;
                        end if;

                    when SPI_HIGH =>
                        if spi_timer = 0 then
                            sclk_reg  <= '0';
                            rx_shift  <= rx_shift(30 downto 0) & miso_sync;
                            spi_bits  <= spi_bits - 1;
                            spi_timer <= SCLK_HALF_CLKS - 1;
                            spi_state <= SPI_LOW;
                        else
                            spi_timer <= spi_timer - 1;
                        end if;

                    when SPI_LOW =>
                        if spi_timer = 0 then
                            if spi_bits = 0 then
                                spi_timer <= SCLK_HALF_CLKS - 1;
                                spi_state <= SPI_HOLD;
                            else
                                sclk_reg  <= '1';
                                mosi_reg  <= tx_shift(31);
                                tx_shift  <= tx_shift(30 downto 0) & '0';
                                spi_timer <= SCLK_HALF_CLKS - 1;
                                spi_state <= SPI_HIGH;
                            end if;
                        else
                            spi_timer <= spi_timer - 1;
                        end if;

                    when SPI_HOLD => -- CS hold time after the last edge
                        if spi_timer = 0 then
                            cs_n_reg  <= '1';
                            mosi_reg  <= '0';
                            spi_timer <= CS_GAP_CLKS - 1;
                            spi_state <= SPI_GAP;
                        else
                            spi_timer <= spi_timer - 1;
                        end if;

                    when SPI_GAP => -- minimum CS-high time before the next frame
                        if spi_timer = 0 then
                            spi_done  <= '1';
                            spi_state <= SPI_IDLE;
                        else
                            spi_timer <= spi_timer - 1;
                        end if;
                end case;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Sequencer: init/verify the INA229, then sample on a fixed period.
    -- Any failed check -> sensor_fault, wait RETRY_CLKS, re-init from scratch.
    ----------------------------------------------------------------------------
    sequencer : process (clk)
        -- Start a frame. 16-bit register = 24-bit frame, 24-bit register = 32.
        procedure issue(cmd   : byte_t;
                        data  : std_logic_vector(23 downto 0);
                        nbits : natural) is
        begin
            spi_tx    <= cmd & data;
            spi_nbits <= nbits;
            spi_start <= '1';
            issued    <= '1';
        end procedure;

        variable ov_now : boolean;
        variable oc_now : boolean;
    begin
        if rising_edge(clk) then
            spi_start <= '0';
            valid_r   <= '0';

            -- Free-running sample clock (keeps Ts fixed regardless of SPI timing)
            if poll_cnt = POLL_PERIOD_CLKS - 1 then
                poll_cnt     <= 0;
                poll_pending <= '1';
            else
                poll_cnt <= poll_cnt + 1;
            end if;

            if reset = '1' then
                seq_state     <= ST_WAIT;
                issued        <= '0';
                wait_cnt      <= STARTUP_CLKS - 1;
                poll_cnt      <= 0;
                poll_pending  <= '0';
                health_cnt    <= 0;
                vbus_code_r   <= (others => '0');
                ishunt_code_r <= (others => '0');
                ready_r       <= '0';
                sensor_flt_r  <= '0';
                ov_r          <= '0';
                oc_r          <= '0';
                ov_cnt        <= 0;
                oc_cnt        <= 0;
                err_cnt       <= (others => '0');
            else
                case seq_state is
                    when ST_WAIT => -- power-up delay or retry back-off
                        if wait_cnt = 0 then
                            seq_state <= ST_RD_MFG;
                        else
                            wait_cnt <= wait_cnt - 1;
                        end if;

                    when ST_RD_MFG =>
                        if issued = '0' then
                            issue(rd_cmd(REG_MFG_ID), x"000000", 24);
                        elsif spi_done = '1' then
                            issued <= '0';
                            if rx_shift(15 downto 0) = MFG_ID_TI then
                                seq_state <= ST_RD_DEVID;
                            else
                                seq_state <= ST_FAIL;
                            end if;
                        end if;

                    when ST_RD_DEVID =>
                        if issued = '0' then
                            issue(rd_cmd(REG_DEVICE_ID), x"000000", 24);
                        elsif spi_done = '1' then
                            issued <= '0';
                            if rx_shift(15 downto 4) = EXPECTED_DIE_ID then
                                seq_state <= ST_WR_CONFIG;
                            else
                                seq_state <= ST_FAIL;
                            end if;
                        end if;

                    when ST_WR_CONFIG =>
                        if issued = '0' then
                            issue(wr_cmd(REG_CONFIG), CONFIG_VALUE & x"00", 24);
                        elsif spi_done = '1' then
                            issued    <= '0';
                            seq_state <= ST_WR_ADCCFG;
                        end if;

                    when ST_WR_ADCCFG =>
                        if issued = '0' then
                            issue(wr_cmd(REG_ADC_CONFIG), ADC_CONFIG_VALUE & x"00", 24);
                        elsif spi_done = '1' then
                            issued    <= '0';
                            seq_state <= ST_VERIFY;
                        end if;

                    when ST_VERIFY => -- read back proves the MOSI path works too
                        if issued = '0' then
                            issue(rd_cmd(REG_ADC_CONFIG), x"000000", 24);
                        elsif spi_done = '1' then
                            issued <= '0';
                            if rx_shift(15 downto 0) = ADC_CONFIG_VALUE then
                                ready_r      <= '1';
                                sensor_flt_r <= '0';
                                health_cnt   <= 0;
                                poll_pending <= '0';
                                seq_state    <= ST_IDLE;
                            else
                                seq_state <= ST_FAIL;
                            end if;
                        end if;

                    when ST_IDLE =>
                        if poll_pending = '1' then
                            poll_pending <= '0';
                            seq_state    <= ST_RD_VBUS;
                        end if;

                    when ST_RD_VBUS =>
                        if issued = '0' then
                            issue(rd_cmd(REG_VBUS), x"000000", 32);
                        elsif spi_done = '1' then
                            issued <= '0';
                            -- VBUS is never negative; all-ones MISO lands here.
                            if rx_shift(23) = '1' then
                                seq_state <= ST_FAIL;
                            else
                                vbus_raw  <= rx_shift(23 downto 4);
                                seq_state <= ST_RD_VSHUNT;
                            end if;
                        end if;

                    when ST_RD_VSHUNT =>
                        if issued = '0' then
                            issue(rd_cmd(REG_VSHUNT), x"000000", 32);
                        elsif spi_done = '1' then
                            issued     <= '0';
                            vshunt_raw <= rx_shift(23 downto 4);
                            if health_cnt = HEALTH_CHECK_POLLS - 1 then
                                health_cnt <= 0;
                                seq_state  <= ST_HEALTH;
                            else
                                health_cnt <= health_cnt + 1;
                                seq_state  <= ST_PUBLISH;
                            end if;
                        end if;

                    when ST_HEALTH => -- catches all-0s MISO and sensor power cycles
                        if issued = '0' then
                            issue(rd_cmd(REG_ADC_CONFIG), x"000000", 24);
                        elsif spi_done = '1' then
                            issued <= '0';
                            if rx_shift(15 downto 0) = ADC_CONFIG_VALUE then
                                seq_state <= ST_PUBLISH;
                            else
                                seq_state <= ST_FAIL;
                            end if;
                        end if;

                    when ST_PUBLISH =>
                        vbus_code_r   <= unsigned(vbus_raw);
                        ishunt_code_r <= signed(vshunt_raw);
                        valid_r       <= '1';

                        ov_now := unsigned(vbus_raw) > OV_LIMIT_CODE;
                        oc_now := signed(vshunt_raw) > OC_LIMIT_CODE or
                                  signed(vshunt_raw) < -OC_LIMIT_CODE;

                        if ov_now then
                            if ov_cnt < FAULT_FILTER then
                                ov_cnt <= ov_cnt + 1;
                            end if;
                            if ov_cnt >= FAULT_FILTER - 1 then
                                ov_r <= '1';
                            end if;
                        else
                            ov_cnt <= 0;
                            ov_r   <= '0';
                        end if;

                        if oc_now then
                            if oc_cnt < FAULT_FILTER then
                                oc_cnt <= oc_cnt + 1;
                            end if;
                            if oc_cnt >= FAULT_FILTER - 1 then
                                oc_r <= '1';
                            end if;
                        else
                            oc_cnt <= 0;
                            oc_r   <= '0';
                        end if;

                        seq_state <= ST_IDLE;

                    when ST_FAIL =>
                        ready_r      <= '0';
                        sensor_flt_r <= '1';
                        if err_cnt /= to_unsigned(255, 8) then
                            err_cnt <= err_cnt + 1;
                        end if;
                        wait_cnt  <= RETRY_CLKS - 1;
                        seq_state <= ST_WAIT;
                end case;
            end if;
        end if;
    end process;
end architecture rtl;
