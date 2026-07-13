# Fork notes — moltenmosfet/stm32-sine (dyno absorber)

This is the Molten MOSFET dyno fork of Open Inverter's `stm32-sine`, driving the
EM57 absorber (decision record Rev 5, 12 Jul 2026). Work orders and rationale live in
`software/stm32-sine_fork_worklist_v1.md`; the findings canon is
`software/stm32-sine_FOC_review_v1.md` (F-numbers).

## Two repos, two forks

`stm32-sine` vendors `libopeninv` as a submodule; each has its own upstream and fork:

| Repo | Upstream (`origin`) | Fork (`fork`) | Baseline pin |
|---|---|---|---|
| superproject | jsphuebner/stm32-sine | moltenmosfet/stm32-sine | `1dfab85` |
| libopeninv    | jsphuebner/libopeninv | moltenmosfet/libopeninv | `78e3f72` |

`libopencm3` submodule is left at its pin (`5ba1bb5`); not forked.

All review line numbers refer to these two pins. `dyno-main` is the integration branch
in each repo, created off the pin and pushed to `fork`. Task branches (`fix/T<nn>-<slug>`)
fork off `dyno-main` and merge back after review.

### libopeninv submodule discipline
A task touching `libopeninv/` commits inside `libopeninv/`, pushes that branch to the
libopeninv fork, then bumps the submodule pointer in the superproject branch. Never let
a superproject branch reference a libopeninv SHA that isn't pushed to the fork.

## Upstream PR policy
`origin` stays pointed at upstream in both repos. `[UPSTREAM]` tasks become clean
branches off *current* upstream master at submission time (rebase then; develop against
the pin now). `[FORK]` tasks encode dyno policy and stay on the fork. Never push to
`origin` (jsphuebner) — forks only. PR sequence: see the worklist's "Upstream PR series".

## Applied tasks

Branch naming `fix/T<nn>-<slug>`, merged `--no-ff` into `dyno-main` in each repo.
Verification: host suite = `cd test && make && ./test_sine`; firmware = `make
CONTROL=FOC` and `make CONTROL=SINE` (ARM toolchain became available mid-wave-2,
so all firmware-only changes are now compile-checked).

| Task | Type | Repo(s) | Verification | Status |
|---|---|---|---|---|
| T0 | FORK | both | n/a | done — forks, remotes, dyno-main, this file |
| T1 | — | superproject | host suite | done — picontroller/foc under test, libopencm3 stubs |
| T2 | UPSTREAM | libopeninv (+super bump) | host tests (3) + FOC/SINE build | done — F2/F7 |
| T3 | UPSTREAM | libopeninv + superproject | host test + FOC build | done — F6 (GetQLimit host-tested; UpdateVoltageLimits firmware-built) |
| T7 | FORK | superproject | FOC/SINE build + host | done — F14 (guarded #if CTRL_FOC) |
| T8 | UPSTREAM | libopeninv (+super bump) | review + FOC build | done — F11 (review-verified per worklist; harness not cheap) |
| T15 | UPSTREAM | superproject | host test + SINE/FOC build | done — F17 |
| T17 | UPSTREAM | superproject | FOC/SINE build + host | done — F19/F20/F21 (3 commits) |
| T4 | FORK | superproject | host tests (7) + FOC/SINE build | done — F1 (QClamp policy class, `qlimfrq` param id 165, 0 = dyno mode / restriction off; shrinking qlimit clamps instantly; dead QLIMIT_FREQUENCY macro removed) |
| T5 | FORK→UPSTREAM cand. | libopeninv + superproject | host tests (4) + FOC/SINE build | done — F10 (`dtcomp` param id 166, default 0 = off; signs captured in ParkClarke, applied in InvParkClarke before short-pulse suppression; 2 A deadband; pinswap resolved at call site) |

Baseline pins → current `dyno-main` tips: superproject `1dfab85 → dyno-main`,
libopeninv `78e3f72 → 5909fdd`.

(Append one row per task as branches merge to `dyno-main`.)
