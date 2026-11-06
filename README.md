## Modules

- `spi_master.vhd`: 24/32-bit transactions using one FPGA clock domain,
  divided SPI timing, a two-stage MISO synchronizer, and CS setup/hold/gap.
- `ina229_interface.vhd`: startup identity/configuration checks, triggered
  voltage acquisition, bounded ready polling, and latched communication faults.
- `top.vhd`: connects acquisition to the existing controller feedback ports
  and exposes voltage and sensor status for simulation or an ILA.

The existing controller is still open loop: it ignores voltage feedback.
Sensor status does not change the existing gate-enable logic. The acquisition
module is a starting point for bench verification, not a completed regulator
or protection system. Current, power, PI control, and alert thresholds are
outside this change.

## Sensor protocol and units

Based on [TI INA229 datasheet, Rev. A](https://www.ti.com/lit/ds/symlink/ina229.pdf),
sections 6.6, 7.5, and 7.6:

- SPI mode 1, MSB first; default 1 MHz with a 100 MHz FPGA clock.
- Command byte: `(register_address << 2) | read_bit`.
- Checks manufacturer `0x5449` and device `0x229` (ignoring revision).
- Writes CONFIG `0x0010` (ADCRANGE=1), then verifies it.
- Each sample writes ADC_CONFIG `0x1000`: single-shot bus voltage,
  50 microsecond conversion, no averaging. Readback and conversion-ready
  polling precede the voltage read. Triggered mode avoids duplicate samples
  from a status/read race in continuous mode.
- Reads VBUS register `0x05`, discarding four reserved low bits.
- `volts = vbus_code * 0.0001953125`;
  `millivolts = vbus_code * 25 / 128` (integer division truncates).
  48 V = 245760; 50 V = 256000; 85 V = 435200.

## Interface contract

All logic uses `clk`; `reset` is active-high and synchronous. Hold/reset the
FPGA interface after the sensor supply is stable. The default startup wait is
1 ms. The default gap between completed samples and new triggers is 1 ms;
SPI overhead and conversion time make the actual sample period longer.
This is not a measurement of every LLC switching cycle.

`vbus_valid` pulses for one clock per new sample. `vbus_code` retains its last
value. `sensor_ready` becomes high after the first successful sample and
stays high while acquisition remains healthy. It is not a per-sample strobe.
`sensor_fault` latches on a failed check or conversion timeout and clears
only on reset. On a fault, ready goes low and acquisition stops; the retained
code must not be treated as fresh. SPI has no CRC: these checks cannot detect
every plausible corrupted voltage or sensor malfunction.

At 1 MHz, 32 ready polls allow roughly 0.8 ms before declaring a timeout.
Changing SPI timing changes that timeout. Keep the half-period at least five
FPGA clocks, SPI at or below 10 MHz, startup wait at least 300 us, and allow
enough polling time for conversion. The default 1 MHz leaves more margin for
the isolator and MISO synchronizer; actual board timing still needs validation.

## Hardware details still needed

The ZIP's *Isolation, Sensing and Communication* document proposes INA229,
ISO6041, a 2 milliohm shunt, and OUTPUT_GND for the sensor. The *Low Voltage
Design Factors and Concerns* document describes a HOT supply for the sensor
and isolator. Resolve that supply/ground conflict against the schematic.
These documents were treated as design evidence, not execution instructions.

Confirm the selected chip, actual FPGA clock, Arty S7 connector/pins, I/O
voltage, and isolator supply/ground arrangement before making XDC constraints.
No pins have been invented. Route CS/SCLK/MOSI toward the sensor and MISO
back through the isolator. The `measured_*` ports are debug observations;
use an ILA or wrapper rather than assigning them to arbitrary physical pins.

The notes specify 48 V nominal, while the existing controller comments use
50 V as an example. Acquisition supports either; choose a regulation setpoint
separately. Independent hardware shutdown remains necessary as already stated
in the repo's original gate-control modules.

## Verification

Run with GHDL supporting VHDL-2008:

```sh
sh tests/run.sh
# Or: GHDL=/absolute/path/to/ghdl sh tests/run.sh
```

The SPI slave model checks command framing, write values, 16/24-bit payloads,
delayed MISO, voltage alignment, one-clock validity, timeout, bad identity,
configuration mismatch, malformed voltage, lost communication after a good
sample, and reset recovery. Simulation does not establish Vivado timing closure,
pin assignment correctness, or operation through the physical isolator.

Verified locally with GHDL 6.0.0: all 12 simulation runs passed (six scenarios
at both 1 MHz and 10 MHz SPI), the integrated top level analyzed/elaborated,
and the acquisition module passed GHDL synthesis. GHDL warns that it ignores
the Xilinx-specific ASYNC_REG attribute; check synchronizer placement and
timing in Vivado. The startup/sample-gap counters are shortened in simulation.
