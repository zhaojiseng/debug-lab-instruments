---
name: debug-lab-instruments
description: Control and debug a model-profiled SIGLENT SDG1032X, SDG2122X, or compatible SDG generator together with a RIGOL DHO924, MHO934, or compatible DHO800/DHO900/MHO900 oscilloscope through bundled terminal-only Python LAN-SCPI controllers. Use for device discovery, dynamic-IP recovery, connection testing, status inspection, SDG model-limit validation, generator or MHO AFG waveform/output control, oscilloscope channel/acquisition/timebase/trigger/measurement control, waveform or screenshot capture, DHO waveform-preview -200 errors, SCPI troubleshooting, simulated regression testing, and coordinated bench signal-chain debugging.
---

# Debug Lab Instruments

Use the bundled terminal-only controllers under `scripts/` unless the user explicitly supplies a project copy. Keep real-device work deliberate, observable, and reversible.

## Route references

Read [references/safety-and-workflow.md](references/safety-and-workflow.md) before every real-device task.

Then read only the device references needed:

- Read [references/sdg2122x.md](references/sdg2122x.md) for the SIGLENT generator.
- Read [references/dho924.md](references/dho924.md) for the RIGOL oscilloscope.
- Read both for end-to-end signal-chain debugging.

## Locate the controllers

Resolve these bundled files relative to this `SKILL.md`:

```text
scripts/sdg2122x_control.py
scripts/dho924_control.py
scripts/test_sdg2122x_control.py
scripts/test_dho924_control.py
```

Run commands from `scripts/`. If the user supplies another controller project, use that copy and run its tests instead. Use Python 3.10+ and the current terminal; do not open a separate GUI window.

The public bundled controllers use RFC 5737 TEST-NET addresses as harmless defaults. Treat them only as placeholders; use an explicit user IP, a cached device, or manual discovery for real instruments.

## Classify the requested action

Classify each operation before sending commands:

1. **Read-only**: discovery, `*IDN?`, status, measurements, error queries, and already-configured waveform inspection.
2. **Configuration write**: frequency, amplitude, channel scale, timebase, trigger, acquisition, labels, network settings, or other parameters.
3. **State/output action**: generator output ON/OFF, RUN/STOP/SINGLE, AUTOSET, reset, arbitrary-wave upload, or network apply.

Perform read-only diagnostics without extra permission when they are relevant. Perform writes and state actions only when the user requested the change. Never infer permission to enable a generator output merely from a request to inspect or prepare settings.

## Select the control surface

- Prefer CLI for repeatable one-shot actions and evidence capture.
- Prefer Python API for multi-step automation, loops, data processing, or coordinated two-instrument work.
- Use TUI only when the user wants interactive terminal control.
- Use raw SCPI only when no structured method covers the command; verify the model, syntax, and error queue first.
- Use the local simulator tests for write-path validation. Do not test new write logic against real instruments.

## Resolve the device safely

1. Prefer an explicit user-supplied IP when present.
2. Otherwise let the controller try its last-device cache.
3. Allow scanning only when requested or when initial automatic location is part of the task.
4. During any scan, show progress and require manual device selection, even for one result.
5. Never auto-select the first scan result.
6. Treat explicit `--host` as strict. If it fails, do not fall back to another device.
7. On connection failure, report the error. Do not automatically start another scan.

Remember that both instruments may use DHCP. Treat documented IPs as current defaults, not permanent identities. Match a moved device by manufacturer, model, and serial number.

## Execute a single-instrument task

1. Run `idn` first unless a valid identity was just obtained in the same session.
2. Confirm that the selected identity matches the command family and inspect the SDG model profile before generator writes.
3. Read the relevant current state.
4. Apply the smallest requested change.
5. Read back the actual target channel or subsystem.
6. Query the error queue when a write is rejected, ambiguous, or uses raw SCPI.
7. Report the resulting state, not only that a socket write succeeded.

## Execute a two-instrument signal-chain task

Use this order unless the user specifies a different safe sequence:

1. Identify both devices and read their current state.
2. Keep the generator output OFF while changing waveform parameters.
3. Configure the generator load, waveform, frequency, amplitude, offset, and phase.
4. Configure the oscilloscope channel/probe ratio, vertical scale, timebase, and trigger only as requested.
5. Read back both configurations.
6. Enable generator output only when the user explicitly requested live output and the stated connection is safe.
7. Measure or capture the waveform on the oscilloscope.
8. Compare requested and measured frequency, amplitude, offset, and waveform shape with units and tolerances.
9. Disable generator output after a temporary test when the user requested shutdown or when the agreed workflow calls for it.

Do not silently compensate for amplitude mismatch. First check Vpp/Vrms/dBm units, generator load setting, scope probe ratio, 50-ohm termination, and channel attenuation.

## Verify changes

Use all applicable evidence:

- Structured status readback.
- Target-channel readback after each write.
- `SYST:ERR?` or the device-specific error queue.
- Oscilloscope measurement values.
- Saved CSV waveform or screenshot when requested.
- Local simulator regression tests after code changes.

For DHO waveform-transfer fixes, also verify the binary payload length, a follow-up text query, an empty error queue, and unchanged channel display state.

Do not claim full success when only the TCP send completed.

## Modify the controllers

When asked to fix or extend controller code:

1. Preserve unrelated user changes and the other instrument controller.
2. Use `apply_patch` for edits.
3. Add or update simulator tests before real-device validation.
4. Run both suites:

```powershell
python -m unittest -v test_sdg2122x_control.py
python -m unittest -v test_dho924_control.py
```

5. Use only read-only real-device checks unless the user explicitly authorizes the new write behavior.
6. Keep all interaction terminal-based; do not add a separate window.

## Handle unsupported devices and functions

- Allow discovery to list generic SCPI devices.
- Restrict non-SDG devices to generic SCPI when using the generator controller.
- Restrict non-DHO800/DHO900/MHO900 devices to generic SCPI when using the scope controller.
- Do not expose DHO924S-only AFG or Bode functions as DHO924 features.
- Expose MHO900 AFG functions only after confirming the model and installed AFG option; treat AFG output ON as a physical output action.
- For SDG1032X, enforce 30 MHz SINE/SQUARE, 12.5 MHz PULSE, 500 kHz RAMP, 6 MHz DDS ARB, and 16 kpts arbitrary-memory limits.
- Do not send `NOISE_ADD`, `MSW`, `SRATE`, `CASCADE`, `COUP TRDUCH`, `POWER:ON:MODE`, or `KEY` to SDG1032X through structured APIs.
- Do not expose SDG7000A-only features as SDG1032X or SDG2122X features.
- Use the official command manuals and the project protocol documents before experimental raw writes.

## Report results

Include:

- Exact identity and endpoint used.
- Whether actions were read-only or state-changing.
- Requested settings and readback values.
- Measurement results with units.
- Errors, timeouts, or unsupported commands.
- Whether generator output remains ON or OFF.
- Paths of any CSV, screenshot, or modified source files.
