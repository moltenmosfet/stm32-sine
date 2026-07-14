# TempMeas

## Description

`src/temp_meas.cpp` / `include/temp_meas.h`. A static class with one public
function, `TempMeas::Lookup(int digit, Sensors sensorId)`, that converts a raw
ADC reading into a temperature in °C by linear interpolation over a per-sensor
lookup table. Unchanged from upstream in this fork. Built into both SINE and
FOC variants; used by `VehicleControl::GetTemps()` (see `doc/vehiclecontrol.md`)
for both heatsink and motor temperature sensors, on every hardware variant
except the Tesla mux'd-ADC path (which calls `Lookup` per-mux-position with
Tesla-specific sensor IDs) and the Prius heatsink (which uses a bespoke linear
formula instead of a table).

`Sensors` enum (`include/temp_meas.h`) — two disjoint ranges packed into one
enum: indices `0..7` (`TEMP_JCURVE` .. `TEMP_BMWI3HS`, `NUM_HS_SENSORS == 8`)
are heatsink-only sensor types; indices `12..23` (`TEMP_KTY83` ..
`TEMP_TOYOTAGEN2`) are motor sensor types, offset from the heatsink range with
a gap for headroom. `Lookup()` re-indexes motor IDs into the same flat
`sensors[]` array via `sensorId - FIRST_MOTOR_SENSOR + NUM_HS_SENSORS`.

No `param_prj.h` parameters are read inside this file; the caller
(`VehicleControl::GetTemps()`) selects which `Sensors` value to pass based on
`snshs`/`snsm`.

Related artifact: `misc/temp_sensors.ods` is a spreadsheet with one sheet per
sensor (`JCurve`, `KTY81-110`, `KTY83-110`, `KTY84-130`, `Semikron`,
`Tesla_100k`, `Tesla_52k`, `Tesla_LDU_Fluid`, `MBB600`, `Leaf_Heatsink`,
`Leaf`, `Toyota_Motor`, `ToyotaGen2Motor`, `Outlander_Front_Motor`,
`Leaf3_HS`, `FS800R`, `EPCOS B57861`), each carrying embedded chart objects —
it's the source/derivation workbook for the `#define` lookup tables at the
bottom of `include/temp_meas.h`, not something the firmware reads at build or
run time.

## Why?

Most of the supported temperature sensors (NTC/PTC thermistors used across
several OEM motor and heatsink assemblies) have a nonlinear resistance/voltage
curve that doesn't fit a simple formula well enough over their full range. A
table plus linear interpolation between sampled points is cheap to compute on
an STM32F1 and accurate enough given the coarse ADC resolution actually needed
for thermal derating. Two sensors (Prius heatsink, computed inline in
`VehicleControl::GetTemps()`) use a closed-form linear formula instead because
their divider network is linear enough not to need a table.

## Drawbacks

- The table only supports linear interpolation between fixed steps
  (`sensor->step`); tables built with variable step spacing would silently
  misinterpolate — every table in this file uses `sensor->step` as a single
  constant step per table, so the sensor data must have been resampled to
  that constant spacing when the table was generated (see the `.ods`
  workbook, not verified further here — "unclear" whether the source data was
  natively uniform or resampled).
- No bounds/sanity check on `digit` beyond falling off the end of the table
  (returns `tempMin`/`tempMax` when out of range) — a disconnected or shorted
  sensor reads as a plausible extreme temperature rather than a distinct
  fault value.
- `sensorId >= TEMP_LAST` returns `0` (°C) rather than signaling an error —
  a caller passing a bad enum value silently gets a plausible-looking
  temperature instead of a fault.

## Architecture

`sensors[]` is a static array of `TEMP_SENSOR` structs (`tempMin`, `tempMax`,
`step`, `tabSize`, `coeff` [PTC/NTC], and a pointer to the raw digit table),
one entry per `Sensors` enum value, built at compile time from the `#define`d
tables under `#ifdef __TEMP_LU_TABLES` in `temp_meas.h` (only `temp_meas.cpp`
defines `__TEMP_LU_TABLES` before including the header, so the raw tables
aren't visible to other translation units).

`Lookup()` walks the table linearly from index 0 looking for the first entry
whose digit value has crossed the input (`cur >= digit` for NTC, `cur <= digit`
for PTC, since NTC and PTC tables run in opposite ADC-digit directions as
temperature rises). On the first table entry, an out-of-range reading returns
`tempMin` directly. Otherwise it linearly interpolates between the crossing
point and the previous sample: `d - c` where `d` is the temperature at the
crossing index and `c` is the fractional offset scaled by `step`. Falling off
the end of the loop (input never crosses) returns `tempMax`.

Called once per `Ms10Task` cycle (10 ms) per sensor, from
`VehicleControl::GetTemps()` and (BMW i3 heatsink path) `ProcessUdc()`.

## Stability

Not covered by the host test suite; unchanged from upstream. Not
hardware-validated in this fork (no fork changes to validate).
