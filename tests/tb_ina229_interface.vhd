library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

entity tb_ina229_interface is
    generic (SCENARIO : natural := 0; HALF_PERIOD : positive := 5);
end entity;

architecture sim of tb_ina229_interface is
    signal clk : std_logic := '0';
    signal reset : std_logic := '1';
    signal miso : std_logic := '0';
    signal cs, sclk, mosi, valid, ready, fault : std_logic;
    signal code : unsigned(19 downto 0);
    signal inject : boolean := true;
    signal samples : natural := 0;
    type codes_t is array(natural range <>) of natural;
    constant CODES : codes_t := (245760, 256000, 435200, 0);
begin
    clk <= not clk after 5 ns;
    dut : entity work.ina229_interface
        generic map (SPI_HALF_PERIOD_CLKS => HALF_PERIOD, POWERUP_CLKS => 100,
                     SAMPLE_GAP_CLKS => 100, MAX_READY_POLLS => 32)
        port map (clk => clk, reset => reset, spi_miso => miso,
                  spi_cs_n => cs, spi_sclk => sclk, spi_mosi => mosi,
                  vbus_code => code, vbus_valid => valid,
                  sensor_ready => ready, sensor_fault => fault);

    -- Independent serial slave: accepts MOSI on falling edges and only
    -- changes MISO after rising edges, including 10 ns return-path delay.
    sensor : process
        variable command : std_logic_vector(7 downto 0);
        variable payload, response : std_logic_vector(23 downto 0);
        variable cfg : std_logic_vector(15 downto 0) := x"0000";
        variable adc : std_logic_vector(15 downto 0) := x"0000";
        variable width : positive;
        variable triggered : time := 0 ns;
        variable prior_cs : time := 0 ns;
    begin
        wait until falling_edge(cs);
        assert sclk = '0' report "SCLK must idle low" severity failure;
        assert now - prior_cs >= 50 ns report "CS gap too short" severity failure;
        command := (others => '0');
        for i in 7 downto 0 loop
            wait until rising_edge(sclk);
            miso <= '0' after 10 ns;
            wait until falling_edge(sclk);
            command(i) := mosi;
        end loop;
        response := (others => '0');
        width := 16;
        case command is
            when x"F9" =>
                response(15 downto 0) := x"5449";
                if inject and SCENARIO = 1 then response := (others => '1'); end if;
            when x"FD" => response(15 downto 0) := x"2291";
            when x"00" | x"04" => null;
            when x"01" =>
                response(15 downto 0) := cfg;
                if inject and SCENARIO = 2 then response(4) := '0'; end if;
            when x"05" => response(15 downto 0) := adc;
            when x"2D" =>
                response(0) := '1';
                if now - triggered >= 50 us and not (inject and SCENARIO = 3) then
                    response(1) := '1';
                end if;
            when x"15" =>
                width := 24;
                response := std_logic_vector(to_unsigned(CODES(samples mod 4)*16, 24));
                if inject and SCENARIO = 4 then response(0) := '1'; end if;
            when others => assert false report "Unexpected SPI command" severity failure;
        end case;
        if inject and SCENARIO = 5 and samples > 0 then
            response := (others => '1');
        end if;
        payload := (others => '0');
        for i in width-1 downto 0 loop
            wait until rising_edge(sclk);
            miso <= response(i) after 10 ns;
            wait until falling_edge(sclk);
            payload(i) := mosi;
        end loop;
        wait until rising_edge(cs) or rising_edge(sclk);
        assert cs = '1' report "Extra SPI clock in frame" severity failure;
        prior_cs := now;
        if command = x"00" then
            assert payload(15 downto 0) = x"0010" severity failure;
            cfg := payload(15 downto 0);
        elsif command = x"04" then
            assert payload(15 downto 0) = x"1000" severity failure;
            adc := payload(15 downto 0);
            triggered := now;
        end if;
    end process;

    check_samples : process(clk)
        variable was_valid : std_logic := '0';
    begin
        if rising_edge(clk) then
            if reset = '1' then
                samples <= 0;
                was_valid := '0';
            else
                assert not (valid = '1' and was_valid = '1')
                    report "valid must be a one-cycle pulse" severity failure;
                if valid = '1' then
                    assert fault = '0' and ready = '1' severity failure;
                    assert code = to_unsigned(CODES(samples mod 4), 20)
                        report "Voltage data/bit alignment incorrect" severity failure;
                    samples <= samples + 1;
                end if;
                was_valid := valid;
            end if;
        end if;
    end process;

    stimulus : process
    begin
        wait for 100 ns;
        wait until falling_edge(clk);
        reset <= '0';
        if SCENARIO = 0 then
            wait until samples = 4;
            assert fault = '0' severity failure;
        else
            wait until fault = '1';
            assert ready = '0' and valid = '0' severity failure;
            if SCENARIO = 5 then assert samples = 1 severity failure;
            else assert samples = 0 severity failure; end if;
            wait for 10 us;
            assert fault = '1' and cs = '1' and sclk = '0'
                report "Fault must latch and stop acquisition" severity failure;
            -- Reset must allow recovery from every injected failure.
            wait until falling_edge(clk);
            reset <= '1'; inject <= false;
            wait for 100 ns;
            wait until falling_edge(clk);
            reset <= '0';
            wait until samples = 2;
            assert fault = '0' and ready = '1' severity failure;
        end if;
        report "PASS scenario " & integer'image(SCENARIO);
        stop;
        wait;
    end process;
    watchdog : process
    begin
        wait for 2 ms;
        assert false report "Simulation timed out" severity failure;
    end process;
end architecture;
