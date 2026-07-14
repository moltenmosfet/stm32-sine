# Anti-cogging (anticog.h)

## Description
`include/anticog.h` provides one header-only, dependency-free function,
`AntiCogTriangle(uint16_t angle)`, used by the FOC build's anti-cogging
feedforward in `src/pwmgeneration-foc.cpp`. It folds a full electrical
revolution (`angle`, 0..65535 = 0..360°) into a quarter-wave and scales it
into a roughly ±32767 triangle wave.

Called from `GenerateAntiCoggingSignal()` (`pwmgeneration-foc.cpp`), which
adds the `cogph` phase offset, calls `AntiCogTriangle`, scales by the
measured cogging-current magnitude (`MeasureCoggingCurrent()`) and `cogkp`
gain, and clamps to `±cogmax`. The result (`Param::anticog`) is injected as
a d-axis feedforward term in `PwmGeneration::Run()` — FOC build only; SINE
doesn't use anti-cogging.

`param_prj.h` params: `cogkp` (gain, id 159, default 0), `cogph` (phase
offset, id 160, default 0), `cogmax` (output clamp, id 161, default 0).
With `cogkp=0` (default), `GenerateAntiCoggingSignal`'s output is always 0
regardless of `AntiCogTriangle` — the feature is off by default, upstream
and on this fork alike.

## Why?
IPM/salient motors produce periodic reluctance torque ripple ("cogging")
synchronized to rotor angle. Anti-cogging feedforward injects a matching
periodic d-axis current correction to cancel it. A triangle wave is a cheap
approximation matched to the disturbance's fundamental, needing no lookup
table or trig call in the hot control loop.

`AntiCogTriangle` was pulled into its own header (same pattern as
`QClamp`/`RegenTaperHold` — `doc/qclamp.md`/`doc/regentaperhold.md`) so the
triangle-folding math — where the bug below lived — can be host-tested
across its full `uint16_t` input domain independent of `Param::` and the
rest of the FOC loop.

## Drawbacks
- Off by default (`cogkp=0`), and per the header's own comment its
  amplitude estimator (`MeasureCoggingCurrent`, not this file) is known
  self-reinforcing: it measures the min/max spread of `id` over one
  revolution, which includes the injection's own effect on `id`. A
  mis-phased `cogph` doesn't converge — it drives the correction up to the
  `cogmax` clamp. Fixing that estimator is explicitly out of scope here and
  documented as a caveat, not fixed.
- A triangle is a coarse approximation of real cogging ripple, which isn't
  exactly triangular; no harmonics beyond the fundamental are modeled.
- Correctness still depends on `cogph` being tuned to the real mechanical/
  electrical phase offset, which is motor- and mounting-specific.

## Architecture
The function folds `angle` into a quarter-wave via four 90°-quadrant
branches (`< 16384`, `< 32768`, `< 49152`, else), producing `[0, 16384]`,
then computes `4 * (uint32_t)angle - 32767` for the final triangle.

**Fork fix (F5/T11, upstream-tracked)**: the original final step was
`uint16_t antiCog = 4 * angle;` — a 16-bit multiply overflowing and
wrapping `65536 → 0` exactly at the 270° branch boundary (folded angle
16384, raw angle 49151→49152), glitching from the triangle's peak (~32769)
straight to its trough (~-32767) instead of transitioning continuously. The
fix does the multiply in `uint32_t` before narrowing to `int32_t`. See
`FORK_NOTES.md` F5. `test/test_anticog.cpp` asserts no adjacent raw-angle
inputs produce outputs differing by more than the per-step slope (4
digits/step) across the entire `uint16_t` domain.

Fixed-point convention: `angle` is the shared `uint16_t`, 65536/revolution
convention used throughout `PwmGeneration`/`Encoder`; the return value is a
plain `int32_t` in FOC d/q controller digit units, not `s32fp`.

## Stability
Host-tested exhaustively (`test/test_anticog.cpp`, all 65536 angle inputs)
and compile-checked in the FOC build. No bench/hardware validation of the
anti-cogging feature — including whether the fixed triangle actually
improves cogging torque on a real motor — has been done on this fork.
