# hwinit / hwdefs (hardware variants and pin mapping)

## Description

`src/hwinit.cpp` / `include/hwinit.h` (setup functions) and `include/hwdefs.h`
(the `HWREV` enum and hardware-conditional macros). Unchanged from upstream in
this fork. Built into both SINE and FOC variants; the functions here are all
called once from `main()` in `stm32_sine.cpp` (see `doc/stm32_sine.md`'s boot
sequence).

Public functions (called from `main()` in this order): `clock_setup()`,
`rtc_setup()`, `io_setup()` (which internally calls `detect_hw()` and, for
some variants, `ReadVariantResistor()`), `tim_setup()`, `nvic_setup()`,
`pwmio_setup()`, `write_bootloader_pininit()`. `spi_setup()` is called lazily
by `VehicleControl::BmwAdcAcquire()` on first use (BMW i3 variant only).

`HWREV` (`hwdefs.h`): `HW_REV1`, `HW_REV2`, `HW_REV3`, `HW_TESLA`,
`HW_BLUEPILL`, `HW_PRIUS`, `HW_MINI`, `HW_LEAF2`, `HW_LEAF3`, `HW_BMWI3`,
`HW_ZOE`. The detected value is stored in the global `hwRev` (declared
`extern` in `hwdefs.h`, defined in `stm32_sine.cpp`) and read throughout the
firmware — `vehiclecontrol.cpp`, `stm32_sine.cpp`, `hwinit.cpp` itself — to
branch on board-specific behavior. No `param_prj.h` parameters are consumed
directly in this file; `hwRev` itself is written to `Param::hwver` in
`UpgradeParameters()` (`stm32_sine.cpp`).

## Why?

This firmware targets several physically different board revisions (the
original "Mini Mainboard" REV1–REV3, a Blue Pill dev board wiring, and several
OEM-controller-specific pinouts — Prius, Tesla, BMW i3, Nissan Leaf/Zoe) from
one binary where possible. Autodetecting the board at boot via resistor
dividers and floating-pin probes (rather than requiring a build-time board
selection) means one firmware image can be flashed across variants and the
correct pin/timer mapping falls out of runtime detection.

## Drawbacks

- Hardware detection is inherently fragile: `is_floating()` and
  `is_existent()` both rely on the STM32's internal ~30k pull up/down and a
  busy-wait settling loop (`for (volatile int i = 0; i < 80000; i++)`) rather
  than a fixed timer-based delay — the loop count is calibrated to the fixed
  72 MHz clock config, not derived from it. A board with a marginal or
  unpopulated variant resistor can be misdetected; `ReadVariantResistor()`'s
  fallback for an unrecognized resistor value is `HW_MINI`, not an explicit
  "unknown" error.
- `hwdefs.h`'s conditional macros (`OVER_CUR_TIMER`, `OVER_CUR_NEG`,
  `OVER_CUR_POS`, `REV_CNT_IC`/`_OC`/`_CCR`/`_CCR_PTR`/`_SR`/`_DMAEN`/
  `_DMACHAN`, `NORTH_EXC_PORT`/`_PIN`/`_EXTI`) are C ternary expressions on
  `hwRev`, evaluated at every use rather than resolved once — readable at each
  call site but re-evaluates `hwRev ==` comparisons repeatedly rather than
  caching a resolved timer/pin set after detection.
- `write_bootloader_pininit()` erases and reprograms a flash page on every
  boot where the computed CRC differs from what's already there — flash wear
  is bounded by how often `bootprec`/`pwmpol` actually change, but the
  function itself doesn't rate-limit or warn about frequent writes.

## Architecture

`clock_setup()` configures the PLL for 72 MHz from an 8 MHz HSE, sets the ADC
prescaler for a 12 MHz ADC clock, sets NVIC to 16 preemption priority levels
with no subpriority, and enables peripheral clocks for the GPIO ports, USART3,
TIM1–TIM4, DMA1, ADC1/2, CRC, AFIO, CAN1, and SPI1 (SPI1 is enabled
unconditionally even though only the BMW i3 variant uses it).

`detect_hw()` runs a short sequence of GPIO probes to distinguish the
"Mini Mainboard" family (checked first, via a PB3/PC10 cross-connection test)
from the "Olimex"-style REV1–REV3/Prius/Tesla/BluePill family, then falls
through to variant-specific floating-pin tests (`is_existent`, `is_floating`)
to pick among `HW_BLUEPILL`, `HW_REV1`, `HW_PRIUS`, `HW_REV3`, `HW_TESLA`, or
default `HW_REV2`. For the Mini Mainboard family, `ReadVariantResistor()`
reads an injected ADC conversion on a variant-ID resistor divider (channel 15)
before and after enabling an internal pull-up, and buckets the result into
`HW_BMWI3`, `HW_MINI`, `HW_LEAF3`, `HW_ZOE`, or default `HW_MINI` — see the
inline reference to the openinverter wiki's Mini Mainboard hardware-detection
table.

`io_setup()` disables JTOG (keeping SWD) before detection (JTAG pins would
skew the floating-pin reads), calls `detect_hw()`, configures the analog and
digital IO lists via the `ANA_IN_CONFIGURE`/`DIG_IO_CONFIGURE` macros
(`anain.h`/`digio.h`, libopeninv), applies a small number of per-variant pin
overrides (extra current-sense channel on REV1, alternate emergency-stop pull
on Prius, extra digital outputs and error-pin remap on Tesla, alternate
analog/digital lists on BluePill), and starts the ADC.

`tim_setup()` configures `OVER_CUR_TIMER` (a `hwdefs.h` macro that resolves to
`TIM2` on BluePill or `TIM4` otherwise) as a 4-channel edge-aligned PWM timer
used both for the over/undercurrent reference DAC-via-PWM output and, on
Prius, an 8.8 kHz booster-converter drive (double `OCURMAX` period). Output
pin alternate-function mapping also branches on `hwRev`.

`nvic_setup()` sets interrupt priorities: PWM timer update at second-highest
priority, TIM1 break (emergency shutdown) and encoder index pulse (EXTI2) at
highest priority, and the scheduler timer (TIM2 or TIM4, whichever isn't
`OVER_CUR_TIMER` on the given variant) at second-lowest.

`pwmio_setup()` reads the current level of the 6 PWM output pins (to detect
and report the idle output pattern) then reconfigures them as either
alternate-function push-pull outputs or floating inputs depending on
`activeLow` (from `Param::pwmpol`).

`write_bootloader_pininit()` builds a CRC-checked `pincommands` block (drive
pattern for the DC-switch/precharge pins and, if PWM is active-high, a
forced-low pattern for the 6 PWM pins) and writes it to a reserved flash page
the bootloader reads on the next boot, only if the computed CRC differs from
what's already stored there (skips the flash erase/write cycle otherwise).

`spi_setup()` configures SPI1 as master for the BMW i3 external ADC
(`VehicleControl::BmwAdcAcquire()`, see `doc/vehiclecontrol.md`); only invoked
on that hardware path.

## Stability

Not covered by the host test suite (hardware register access isn't
host-testable in this harness — see `doc/host-tests.md`); unchanged from
upstream. Not hardware-validated in this fork.
