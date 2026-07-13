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

| Task | Type | Repo(s) | Branch | Status |
|---|---|---|---|---|
| T0 | FORK | both | dyno-main | done — forks, remotes, dyno-main, this file |

(Append one row per task as branches merge to `dyno-main`.)
