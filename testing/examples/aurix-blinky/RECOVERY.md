# AURIX Lite Kit recovery runbook (TC2x, TC3x, TC4x)

How to get a TC275/TC375/TC4D7 Lite Kit back to a known-good state (blinky
running, no fault LED) using only this repository's own OpenOCD build.
`recover-board.sh` in this directory automates the commands below; read this
file at least once before using it on a board you have not recovered before.

## When to use this

- A board's fault LED is on and/or its blinky has stopped running.
- A board is simply unresponsive to the regression suite or to a manual
  `openocd` invocation (connect errors, timeouts).
- You are not sure the flash content is still correct (e.g. after an
  interrupted flash write).

TC4D7 has a dedicated fault/alarm LED, LED5 (red = fault, off = healthy),
separate from its two blinky LEDs. TC275 and TC375 have no documented
equivalent fault LED in this tree; for those boards "recovery" mainly means
"the blinky LEDs stopped and a reflash should bring them back."

## Ground rules

- **Identify the board by USB serial, every time** -- never trust a
  previously-seen USB location/port number. A power-cycle has been observed
  to swap which location two different boards enumerate at, while the
  serial never changes. Run `recover-board.sh list` first.
- **One board at a time.** Never run a recovery attempt against a board
  while another command (regression suite, GDB session, etc.) is using a
  different board -- not because they interfere with each other, but to keep
  the state of what you've touched this session unambiguous.
- **If `recover-board.sh` fails with an `[OCMTS] UNWEDGE: exhausted N
  reset+reconnect cycles` error, do not run it again immediately.** See
  "If recovery fails" below -- this is the single most important rule in
  this document and is based on a real incident (see Postmortem).
- Never fabricate register/memory addresses or vendor identifier strings;
  everything in this document was read out of files already in this tree or
  obtained live from an attached board.

## §0 -- Identify the board

```bash
cd openocd-gerrit-tricore/testing/examples/aurix-blinky
./recover-board.sh list
```

This reads sysfs directly (no DAP access, zero risk) and prints every
attached Infineon probe's USB location, product string, and serial, e.g.:

```text
loc      product                serial
1-2      DAS JDS AURIX LITE KIT V1.0 (TC4D7) LKBM1OL0
1-3      DAS JDS AURIX LITE KIT V1.1 LK7U8NLP
1-5      DAS JDS AURIX LITE KIT V2.0 (TC375) LKAGI1QM
```

Confirm the board you intend to recover is actually present. The serials
used by `recover-board.sh` are fixed per physical board:

| Script arg | Board                  | Family | Serial     |
|------------|------------------------|--------|------------|
| `tc2`      | TC275 Lite Kit         | TC2x   | `LK7U8NLP` |
| `tc3`      | TC375 Lite Kit         | TC3x   | `LKAGI1QM` |
| `tc4`      | TC4D7 Lite Kit         | TC4x   | `LKBM1OL0` |

(If you ever add a fourth board or a serial changes, update these two places:
this table and the `board_info()` function in `recover-board.sh`.)

## §1 -- Diagnose first (read-only)

```bash
./recover-board.sh tc4 diagnose   # or tc2 / tc3
```

This runs `init` (connect + examine all cores) and a single `reg pc` read,
then `shutdown` -- no reset, no halt, no flash access. A clean result looks
like each core printing `Examination succeed`. This confirms the board is
reachable at all before you commit to reflashing it.

**If this fails with the `UNWEDGE` signature described above, stop and go
straight to "If recovery fails" below** -- do not run `diagnose` again and
do not proceed to `recover`.

## §2 -- Recover

```bash
./recover-board.sh tc4 recover   # or tc2 / tc3
```

This runs the same one-line command each board's own
`testing/examples/aurix-blinky/<family>/README` documents, and the same one
the hardware regression suite itself uses to load the board at the start of
every run:

```text
program <family>-blinky.elf verify reset exit
```

i.e. connect, reset into a known state, erase+program the flash, read the
whole image back and compare it (`verify`), then `reset run` and disconnect.
No ELF-to-binary conversion or physical-address workaround is needed here:
unlike some other OpenOCD trees, this repo's own board configs already
declare the `0xA0000000` non-cached alias as its own flash bank (see
`pfls0_nc`/`pfls00_nc` in `tcl/target/infineon/tc27x.cfg`, `tc37x.cfg`,
`tc4dx.cfg`), so the ELF can be programmed directly at its linked address on
all three boards.

## §3 -- Confirm

Check the board **twice**: once right after `recover` reports success, and
again after waiting about 60 seconds. A single immediate glance is not
enough to catch a relapse -- see Postmortem. Expected: fault LED off (TC4D7)
and the two blinky LEDs blinking continuously in both checks, on all three
boards.

**Do not run any further hardware-touching command -- including another
`diagnose` -- after a successful recovery, beyond what you actually came to
do.** Treat a successful recovery as the end of the session for that board.

## If recovery fails

If `diagnose` or `recover` ever reports the `[OCMTS] UNWEDGE: exhausted N
reset+reconnect cycles` / `is not accessible via IO client after adapter
reset` signature, this means the on-chip debug subsystem's IO client did not
come up cleanly on the very first connect attempt. Retrying the same command
with this driver does not fix it and has twice been observed to turn a board
that was still running fine into one showing a fault LED (see Postmortem).
Escalate instead, in this order:

1. **Physical power-cycle the board** (unplug/replug USB power, or the
   kit's own power switch if it has one), then retry `recover-board.sh
   <board> recover` once, cleanly. This is the simplest fix and has resolved
   every occurrence of this signature seen so far.
2. **If a power-cycle is not possible, or the board still fails the same way
   immediately afterward**, fall back to the vendor DAS stack, which has its
   own, more tolerant reconnect handling. This requires tooling outside this
   repository (Infineon DAS, normally under `/opt/Tools/DAS/<version>/`, and
   a checkout of the vendor `tricore-oss-openocd` tree) -- confirm both are
   available before starting:

   ```bash
   # start the vendor DAS server (seizes every attached Infineon probe on
   # the host for as long as it runs -- get sign-off from whoever else might
   # be using a different board before starting this)
   # keep attached; stop with the same job-control mechanism you started it
   # with, never pkill/killall
   /opt/Tools/DAS/<version>/bin/tas_server_console &
   ss -ltnp | grep 24817   # confirm it's listening before proceeding

   # the vendor adapter driver selects boards by a full display-name string,
   # not the bare USB serial -- read it straight from the "product" column
   # of `recover-board.sh list` (or `lsusb -v`) and drop the "DAS JDS "
   # prefix, e.g.:
   #   TC2x : AURIX LITE KIT V1.1 LK7U8NLP
   #   TC3x : AURIX LITE KIT V2.0 (TC375) LKAGI1QM
   #   TC4x : AURIX LITE KIT V1.0 (TC4D7) LKBM1OL0

   cd <vendor tricore-oss-openocd checkout>
   # TC3x / TC4x have ready-made board configs; TC2x does not (see note below)
   src/openocd -s tcl -f board/infineon/kit_a2g_tc375_lite.cfg \
     -c 'adapter serial {AURIX LITE KIT V2.0 (TC375) LKAGI1QM}' \
     -c init -c "reg pc" -c shutdown      # diagnose first, same reasoning as §1

   # if flash content is in question, reflash via the vendor stack. The
   # vendor board configs do not declare the 0xA0000000 alias bank, so
   # convert the ELF to a raw binary and write it at the physical base
   # address instead -- 0x80000000 for all three families (confirmed in
   # this repo: tc27x.cfg line ~20 states "0xA0000000 is a non-cached alias
   # of 0x80000000"; tc37x.cfg / tc4dx.cfg declare their pfls0/pfls00 flash
   # bank at physical 0x80000000):
   # (requires the tricore-elf toolchain on PATH; see e.g. aurixgcc/bin or
   # tricore-gdb/bin in the top-level tricore tree). Write the output under a
   # working directory you control -- never under /tmp.
   tricore-elf-objcopy -O binary \
   openocd-gerrit-tricore/testing/examples/aurix-blinky/tc375/tc375-blinky.elf \
   tc375-blinky-phys.bin
   src/openocd -s tcl -f board/infineon/kit_a2g_tc375_lite.cfg \
     -c 'adapter serial {AURIX LITE KIT V2.0 (TC375) LKAGI1QM}' \
     -c "program tc375-blinky-phys.bin 0x80000000 verify reset exit"

   # stop the server the same way you started it -- never leave it running
   ```

   Substitute the TC4D7 board config/identifier/ELF for that board the same
   way -- this exact sequence (physical-address reflash of the TC4D7 via the
   vendor stack, including the objcopy-to-binary step) was already validated
   end-to-end on real hardware earlier in this project; see
   `tc4-work-2026/RECOVERY.md` for that original run's log output.

   **TC2x note:** the vendor tree has no ready-made board config for the
   TC275 Lite Kit (it only ships one for the different TC277 AppKit). If
   ever needed, one can likely be built by combining two files that already
   exist in the vendor tree, following the exact same pattern as the TC3x/
   TC4x board configs:

   ```tcl
   source [find interface/tas_client.cfg]
   reset_config srst_only
   adapter speed 5000
   source [find target/infineon/tc27x.cfg]
   ```

   This has **not** been tested in this project. Prefer the power-cycle path
   for TC2x; treat this as a last resort and verify carefully before trusting
   it.

3. After any vendor-stack recovery, **do not reconnect with this
   repository's own driver as a "just to check" step without a physical
   power-cycle first.** Treat that as a separate, deliberately-chosen,
   explicitly risky action if you ever want to do it for some other reason
   -- never as a routine part of recovery. See Postmortem.

## Why the one-liner is enough here

- **TC2x** needs a Boot Mode Header at `0xA0000000` for the chip to run the
  flashed image without a debugger attached. This is generated automatically
  as part of the normal build (`gen-bmhd.py` / `bmhd.S` / `tc27x.ld`), so the
  `program ... verify reset exit` command already includes it.
- **TC3x** needs its CPU and safety watchdogs disabled very early after
  reset, or they expire before the application starts. Already done in the
  blinky's own `startup.S`; not something recovery needs to handle
  separately.
- **TC4D7** needs neither of the above -- empirically confirmed (a
  free-running-counter probe ran 500ms+ with no watchdog reset on real
  hardware; see `tc4d7/startup-stage1.S`).

## Postmortem -- the OCMTS "UNWEDGE" failure mode is real, not theoretical

While developing this document, a read-only `diagnose` run against the
TC4D7 (immediately after the board had been used in a different debug-tool
session, with no power-cycle in between) failed with the `UNWEDGE:
exhausted 8 reset+reconnect cycles` signature. The board's fault LED was
confirmed red immediately afterward. A same-driver `recover` attempt failed
identically. The fix that actually worked was the one this document
recommends: a clean vendor-stack `reset run` (no reflash -- the flash
content was not in question), after which the board was confirmed stable
both immediately and after a 60-second wait.

This matches an earlier, independently-observed incident in this project
(recorded in `tc4-work-2026/RECOVERY.md`): a board that had just been
successfully recovered via the vendor stack relapsed to its fault state
after a single own-driver cross-check (`reset halt`), which hit the same
`UNWEDGE` signature.

The common thread across both incidents is not "which driver is buggy" --
it is **switching debug-tool stacks against the same board without a
power-cycle in between**, combined with retrying the same failing tool
multiple times once the IO client is not ready on the first try. Each retry
is itself an adapter-level reset; several in a row appear able to trip the
chip's safety watchdog/SMU alarm even though no command in the chain ever
asked for a `reset` or `halt` explicitly. Once you see the `UNWEDGE`
signature, stop issuing more commands from the same tool and escalate as
described above instead.

## References

- `recover-board.sh` -- the script this document describes.
- `testing/examples/aurix-blinky/{tc27x,tc375,tc4d7}/README` -- per-board
  build/flash instructions; source of the `program ... verify reset exit`
  one-liner and the family-specific gotchas above.
- `tcl/target/infineon/tc27x.cfg`, `tc37x.cfg`, `tc4dx.cfg` -- flash bank
  declarations; confirm the `0xA0000000`/`0x80000000` alias relationship and
  the alias bank names used by `program`.
- `ab-test/boards.sh` -- sysfs-based board enumeration this script's `list`
  subcommand adapts; also the source of the "USB locations are not stable
  across power-cycles" lesson.
- `tc4-work-2026/RECOVERY.md` -- the original, TC4D7-only, vendor-stack-based
  runbook this document generalizes and supersedes for day-to-day use; kept
  as reference material for the vendor-stack escalation path.
