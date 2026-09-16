# SCL-only watchdog characterization

This is a separate **full-driver** hardware experiment. It does not replace or
lengthen the small one-shot driver's 100 ms deadline test. Checkpoints 4–5
preserve the original production code and its diagnostic failures. The later
production integration adds confirmed local controller/target reset; one-shot
ownership, eDMA implementation and application code remain unchanged.

Baseline result: **120/140 ms produced exact PLTF but failed post-release SCL
checks; 200 ms completed exact data and STOP.** After the local-reset fix,
the default full sweep passes: exact PLTF with MBF idle at return and SCL high
after peer release, plus the completing 200 ms control. BBF may remain set;
this is local-engine recovery, not automatic full bus recovery.
See the checkpoint ledger for the preserved failures and recovery-state evidence.

The receiver accepts the address and withholds RX service for at least 150 ms.
It never pops SRDR or arms RX DMA during the hold. Every approximately 1 ms
sample checks RDF pending, address consumed, no fault/termination, RXSTALL on,
RDDE/ERQ/ACTIVE off and SCL low. Timers and interrupts remain live.

The other board uses the full async/DMA controller at nominal 3 MHz functional
clock and 100 kHz bus rate. Its software progress budget is 500 ms, with a
separate 750 ms whole-call backstop. Neither software timeout is accepted as a
hardware timeout. `MCFGR3[PINLOW]` is rounded upward by the existing driver:

| Requested | Read-back PINLOW | Nominal hardware threshold | Required result |
| --- | ---: | ---: | --- |
| 120 ms | 1407 | 120.064 ms | Exactly `PinLowTimeout`; DMA quiescent, FIFOs empty, MBF idle at return and SCL high after peer release |
| 140 ms | 1641 | 140.032 ms | Exactly `PinLowTimeout`; DMA quiescent, FIFOs empty, MBF idle at return and SCL high after peer release |
| 200 ms | 2344 | 200.021333 ms | Successful write; target receives exact 16 bytes and STOP |

The first two targets explicitly abort their response **after** the 150 ms hold.
This is independent peer cleanup, not controller bus-clear. The 200 ms target
instead resumes receiving into a 17-byte buffer and requires `Stopped(16)` plus
`Hello from DMA!\n`. An abort, NACK, software timeout or other error cannot pass
the completing control. Each case starts from fresh board initialization.

## Diagnostic boundary

The production `PinLowTimeout::Enabled` API selects SCL-or-SDA monitoring. This
probe requires SCL specifically. After fresh driver construction, before any
transaction/lease exists, a short critical section disables the bound IRQ,
asserts MIER/MDER zero and controller idle, disables MEN, writes only TIMECFG to
SCL-only, then enables MEN and the bound IRQ again. Each MEN change has a
bounded 1 ms readback wait: immediate readback retained the previous value on
the bench. No configuration write is allowed before disable is observed.
No live driver future can
overlap it. This is a reviewed PAC escape **in diagnostic firmware**, not a
new safe-driver API or a demonstration of type-enforced configuration.

Readback requires PRESCALE=0 (divide by one), TIMECFG=0 (SCL), FILTSCL=0,
the exact PINLOW above, and the expected functional-clock mux/divider. Initial
PLTF must be clear. The nominal source frequency comes from the HAL clock tree;
it is not an independent frequency measurement. A MEN toggle by itself is not
claimed to start a calibrated timer epoch: fresh construction/reset, idle-high
input and clean initial status are also required.

After return, both controller DMA channels must have ERQ/ACTIVE off, peripheral
DMA requests off, FIFOs empty, and MEN enabled. These checks precede any
diagnostic repair. Raw MCR/MSR/MFSR plus sampled SCL are logged at return and
100 ms later, after the target's scheduled release, without intervening repair
in the default `-Recovery none` mode.
These register reads are not an atomic hardware snapshot. The controller
object is kept alive through semihost exit so its pin-disabling Drop cannot
change the target's still-running hold.

**Recovery qualification is separate.** The original exploratory run obtained
exact PLTF but failed an immediate MBF-idle assertion. RM section 42.3.1.2
(p1902) describes MEN-off as a graceful close, which can wait for peer release;
that assertion was not justified by the original MEN-toggle implementation.
Checkpoint 4 therefore preserved MBF as an observation and failed post-release
SCL checks rather than claiming recovery. After the confirmed abrupt-reset fix,
the default fault cases now require MBF idle at both snapshots and SCL high
after peer release. Historical opt-in repair arms still permit the original
busy return snapshot before their diagnostic intervention. Persistent BBF is
not MBF, and neither local idle nor a threshold PASS establishes that a new
transaction can start: explicit bus recovery is tested separately in the rig.

## Passive sampled observer

`checks/scl_observer.rs` also serves the original RXSTALL target. Its earlier
CC3-only precondition was too restrictive: HAL initialization uses GPIO3's
automatic ACC3 gate. The observer now recognizes software or automatic gating,
checks reset release, Mux2, IBE, no inversion and PID21=0, then requires real
idle-high/held-low/released-high readback. It writes **no** clock, reset, mux,
input gate, GPIO output or direction register and creates no second pin owner.
The watchdog target additionally checks unchanged PCR/PDDR/PDOR/PIDR snapshots.

Hardware basis: local MCXA reference manual Rev.1 Draft F (RC2), 2026-02-02:
section 15.3.3 (p538) permits PDIR reads with PID=0 for a digital function;
section 17.5.2.19 (p772) defines ACC3 automatic gating; section 42.3.1.5 (p1904)
describes pin-low timeout without a pin transition; sections 42.7.1.9/11
(p1931/1934) define TIMECFG/PRESCALE and PINLOW. The gate fields plus the HAL's
configuration and actual readback support this selected-board observer; they
are not a newly derived universal CC/ACC clock truth table.

## Limits and timing margins

A passing fault case is a **hardware-qualified SCL threshold observation**:
the peripheral's synchronized/filtered input met its configured low condition.
It is not a glitch-resolved waveform, exact edge timing, or an exact interval
for one event reconstructed from separate runs. Controller terminal recovery
changes the bus after PLTF; only the already-crossed threshold is evidence.

OSTIMER uses FRO12M-derived `clk_1m`; the controller and target have independent
oscillators. The local manual p1091 describes percent-level accuracy and defers
the guaranteed limits to the datasheet. No exact MCXA577 tolerance was verified
for this checkpoint. Values here are nominal, not calibrated SI-time bounds or
qualification across voltage, temperature and silicon. The 200 ms control adds
margin over 160 ms; no numerical +/-2% or +/-3% guarantee is assumed. The 140 ms
trip still has only 10 ms nominal headroom before target resumption, so a future
failure must be investigated rather than retried into PASS or called a driver
regression without checking clocks and scheduling.

## Run and evidence

From `examples/mcxa5xx`, PowerShell 7:

```powershell
./one-shot/tests/test-watchdog-verdicts.ps1
./one-shot/run-watchdog.ps1 -ControllerProbe '1fc9:0143:YOUR_CONTROLLER_SERIAL' -TargetProbe '1fc9:0143:YOUR_TARGET_SERIAL'
```

This flashes both boards, once per case, and rebuilds/runs the normal pair in
`finally`, including on failure. A 20 s controller startup window and a
conservative 19 s host READY budget include flashing time; they do not alter
the transfer or hold thresholds. Source state, ELF/log hashes, complete tails,
configuration, measured timing and restoration outcomes go to
`target/i2c-one-shot-watchdog/<run-id>/verdict.json`. Both zero exits and exact,
unique, mode-specific markers are required. Missing/contradictory/malformed
telemetry fails closed. Historical failures are not overwritten or hidden.
An explicit `-Modes 140,200` selects a subset and records `full_sweep=false`;
empty or duplicate selections are rejected. After a failure the runner may
reset both boards and execute the next distinct requested case, never retry
the failed case. Failure remains sticky even if all later cases pass.
Failed cases retain complete raw logs and hashes; validated observation objects
are not manufactured from logs that failed acceptance. Their raw snapshots are
transcribed with the failure in PROGRESS.md.

See [PROGRESS.md](PROGRESS.md) for actual run outcomes; this design description
is not itself a passing hardware verdict.

## Production reset integration

The shared private reset sequence writes a complete RST=1/enable=0 control word,
observes reset assertion, releases reset while disabled, observes that state,
and only then restores configuration. Its non-Copy `Disabled` owner borrows the
same register adapter through mode configuration and enable. Dependent writes
cannot follow a failed confirmation through this API. Each handshake is bounded
by 1 ms and a poll-count ceiling, independent of peer release and even if the
time source stops. Unexpected handshake failure leaves the last attempted word
holding reset; the facade fails before resolving its recovery proof.

Controller setup, PLTF recovery, last-resort reset and bus-clear handoff use it;
target setup/abort use the same sequence with target-specific configuration.
Ordinary FIFO/halt operations are not converted into whole-engine resets. DMA
leases must already be quiescent; this primitive does not replace their buffer
ownership or quiescence protocol. The one-shot's existing destructive teardown
remains unchanged.

Tests execute this actual sequence through individual register-read/write
effects, including delayed/stuck readback, destroyed/nonzero configuration,
mode changes, timeout fail-stop and a stopped clock. The mutation manifest also
removes reset-release confirmation and requires the specific effect-order
assertion to fail. These tests cover the sequence, not electrical waveforms or
the correctness of the small PAC/Tock adapter maps, which remain reviewed.

## Post-return repair comparison (opt-in)

Checkpoint 4 is committed independently as `77a53f66d`. Its failed recovery
evidence remains the baseline. `-Recovery wait` and `-Recovery reset` select
additional **diagnostic-only** PAC operations after the original production
`PinLowTimeout` return. They accept only `-Modes 120,140`; neither changes the
original threshold/hold assertions or the target's designated abort.

Both arms mask the peripheral IRQ, require DMA already quiescent, write MEN=0
as a complete word preserving only DOZEN/DBGEN, and await MEN=0/RST=0 readback
with a 300 ms deadline. This wait yields; it is not a critical-section spin.
Only the reset arm then pulses RST and restores all eight master configuration
registers, checking exact readback. ABORT must be absent and is never replayed;
interrupt/DMA enables, live commands and status are not restored. The wait arm
checks that the same configuration stayed unchanged.

**Both arms remain disabled until the same return+100 ms checkpoint.** Missing
that checkpoint fails the matched comparison even if the readback deadline has
not expired. At it, separately require MEN=0, MBF idle, empty FIFOs and SCL high.
Only then clear observed status, re-enable MEN, confirm readback and perform a
new write of `Recovered DMA!!\n`. The target must see a new address, exactly
those 16 bytes and STOP. No extra target reset and no transfer retry is allowed.
The runner additionally requires ordered, unique repair records on both CPUs;
normal mode rejects repair records. It still restores the ordinary pair in
`finally` after a failed case.

Interpretation is deliberately limited. If both arms succeed, confirmed
disable/dwell suffices for these conditions. If only reset succeeds, reset adds
recovery capability here. If neither succeeds, investigate the endpoints
separately. **None proves that stale RMW caused the original failure:** the
original shutdown already happened, and both repairs also change control-write
shape and disabled dwell. MEN=0 is not MBF idle or proof of peer release.

Likewise the ~0.226 ms excess above nominal PINLOW thresholds is not a measured
FRO tolerance: controller OSTIMER and its watchdog share a FRO-derived source.
Detection, phase and software latency remain. Independent target/controller
clock error affects the hold-versus-threshold margin, not that claimed
oscillator measurement. The separate 200 ms completing control does not create
a precise single-event timing bracket.

`-Recovery abrupt` is a separate production-shape experiment, not another
matched-dwell arm. It writes RST=1/MEN=0 immediately, confirms that state, then
releases reset and confirms disabled readback, all within 1 ms. `reset_wait_us`
measures this reset sequence, not graceful-close latency. It restores the same
configuration and re-enables immediately while SCL must still be low; the
`reset_complete` record requires MEN=1, MBF idle and empty FIFOs. At return+100 ms
it still requires high/idle and the same fresh exact transfer. This distinguishes
bounded local engine recovery from post-release bus usability.

**Historical diagnostic result: abrupt reset restores local idle in 11–12 us, but the fresh transfer
fails.** MSR remains `0x02000001` (BBF set, MBF clear) after SCL rises. The
runner correctly fails both cases. SCL alone did not establish SDA high in this
arm; the following `abrupt_idle` arm added that observation and separated
physical inputs from bus-idle history before production integration.

`-Recovery abrupt_idle` adds one explicit diagnostic policy change to abrupt
reset: BUSIDLE is enabled while MEN=0, with every other configuration bit
preserved. On these selected timings (CLKLO=14, SETHOLD=14, CLKHI=12), the RM
minimum `2*(CLKLO+SETHOLD+2)` is 60, also greater than CLKHI+1. At 3 MHz and
PRESCALE=0 this recognizes BOTH lines high for nominal `(60+1)/3 MHz`, about
20.33 us. It does not use PINLOW's factor of 256. Exact readback and derived
bounds are checked; the generic 6-bit timing limits alone are not a guarantee
that another controller cannot hold a high period longer.

This arm additionally validates the SDA input path and samples SDA/SCL from one
passive PDIR read after the matched wait. Both physical inputs must be high and
BBF clear before the fresh transfer. Both 120/140 cases PASS with this policy,
while the otherwise matching zero-BUSIDLE abrupt cases failed. That supports
bus-idle recognition as the missing capability in these cases, not physical
bus-clear or proof of the original stale-RMW mechanism.

**Not a new global default.** A short automatic idle detector changes shared-bus
policy and may mistake another controller's long high period for idle. It does
not release a line held low and it does not generate STOP. Production local
reset preserves the configured BUSIDLE; the existing explicit `clear_bus`
API remains available to a caller authorized to recover the bus. Any automatic
single-controller policy should be separately opt-in and documented.
