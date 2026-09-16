# Withheld receive service: a separate hardware probe

This exercises the small async/DMA controller's deadline against a deliberately
unserviced receiver. It changes neither the normal 33-line sending example nor
the production controller, target or eDMA implementation.

The target first accepts the address, then waits for a real received byte
without arming RX DMA or reading SRDR. This releases address stretching before
the hold; it is not a guessed delay before the target started. Every hold sample
requires RDF pending, address consumed, no fault/STOP/restart, RXSTALL enabled,
RDDE off, and RX channel 1 ERQ/ACTIVE off. Timers and interrupts keep running.

| Case | Target action | Required result |
| --- | --- | --- |
| Short | Withhold service for at least 5 ms, then arm RX DMA | Exact 16 bytes and `Stopped(16)` in a 17-byte buffer; controller success and local cleanup |
| Long | Withhold service for at least 150 ms, then explicitly abort the target response | Exactly controller `Timeout` at 95–125 ms; ERQ/ACTIVE/TDDE/MEN off and I2C IRQ disabled before any repair |

The long target abort is a **separate peer action**, not controller bus-clear.
The probe never claims that a controller timeout freed the target or proved an
application-level device operation complete. Target waits are bounded, and
observation/timeout failures abort the response before reporting failure.

## What is—and is not—measured

Peripheral status, DMA inactivity, local elapsed time, exact data/STOP and
cleanup are measured. The 1 ms sampling loop does not claim to observe every
instant; its sample count can vary with scheduling.

The optional read-only SCL observer first checks PORT3/GPIO3 clock gates and
reset release, P3_21 mux,
input enable, inversion and GPIO input-disable state. If usable, it requires
idle-high, low hold samples, and high after release. It never remuxes a pad,
creates another GPIO owner, or enables clocks to make the observation pass.

At checkpoint 3, CC3.GPIO3=0 made the original CC-only observer report
unavailable without reading PDIR. HAL init actually uses GPIO3's ACC3 automatic
gate. The shared observer now recognizes that path without writing clocks or
input configuration; it still requires actual high/low/high readback. Historical
unavailable verdicts remain unchanged. No RX-service, data/STOP, timeout or
cleanup assertion is skipped when an optional observer is unavailable.

Discrete samples cannot establish continuous SCL on their own. The separate
[SCL-only watchdog experiment](WATCHDOG.md) can supply a hardware-qualified
low-threshold witness, subject to input sampling and clock tolerance, without
an external analyzer. An analyzer is still needed for the waveform itself;
neither method alone qualifies timing margins across operating conditions.

## Run

Use PowerShell 7, the repository's Rust toolchain, `flip-link` and `probe-rs`.
Use the existing P3_20 SDA / P3_21 SCL / common-GND wiring and pull-ups.
From `examples/mcxa5xx`:

```powershell
./one-shot/tests/test-rxstall-verdicts.ps1
./one-shot/tests/test-rxstall-startup.ps1
./one-shot/run-rxstall.ps1 -ControllerProbe '1fc9:0143:YOUR_CONTROLLER_SERIAL' -TargetProbe '1fc9:0143:YOUR_TARGET_SERIAL'
```

The runner builds both dedicated binaries for `short`, then `long`, with locked
dependencies and warnings denied. `I2C_ONE_SHOT_RXSTALL` selects the mode; absent
or unknown values fail at boot. The runner owns both probe processes, resets
both boards between cases, checks startup timing and requires exact markers
plus exit zero from both endpoints. No failed mode is retried into a PASS.

Programming and verifying both ELFs is a bounded preparation step outside the
startup window. The subsequent `run --preverify --verify` still verifies and
freshly starts each board. The unchanged 9 s host readiness budget begins
before the controller launch, and the target must publish READY inside it,
before the controller's 10 s quiet window ends. Slow setup does not enlarge any
transfer deadline or acceptance window; preparation and startup timings are
recorded separately. A failed preparation still invokes normal restoration.

After board use, a `finally` block rebuilds and runs the normal pair, including
after a failed probe. Successful restoration never overwrites a probe failure.
Environment values are restored. Logs, source state, selected probes, ELF hashes,
observations and verdicts are saved under `target/i2c-one-shot-rxstall/<run-id>/`.
This runner is Windows/PowerShell-specific; no cross-platform execution claim.

## Read the code

- [Controller checks](checks/rxstall-controller.rs): real one-shot transfer,
  exact result and elapsed window, then read-only local cleanup audit.
- [Target checks](checks/rxstall-target.rs): accepted address, unserviced RX,
  short resume or long abort, optional sampled SCL witness.
- [Runner](run-rxstall.ps1) and [verdict controls](tests/test-rxstall-verdicts.ps1):
  fail-closed host orchestration, separate from firmware invariants.

See [the checkpoint ledger](PROGRESS.md) for actual committed validation and
the preserved exploratory failure; this description alone is not evidence.
