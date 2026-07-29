# Safety and coordinated debugging workflow

## Contents

1. Permission model
2. Connection discipline
3. Read-only baseline
4. Coordinated generator/scope workflow
5. Measurement mismatch diagnosis
6. Failure handling
7. Code-change validation

## 1. Permission model

Treat the following as read-only:

- Network reachability checks.
- Device discovery using `*IDN?`.
- `idn`, `status`, measurement, and error queries.
- Reading cached device identities and endpoints.
- Reading existing waveform data when it does not require changing acquisition mode.

Treat the following as configuration writes:

- Generator waveform, frequency, amplitude, offset, phase, load, modulation, sweep, burst, arbitrary wave, coupling, and network parameters.
- Scope channel display/scale/offset/coupling/probe, acquisition, timebase, trigger, DVM, counter, digital, display, and system parameters.

Treat the following as higher-risk state actions:

- Generator output ON.
- MHO900 built-in AFG output ON.
- Scope AUTOSET or reset.
- Any reset, network apply, arbitrary-wave upload, or firmware/model-experimental raw command.
- Scope RAW memory acquisition if it requires STOP.

Only perform a write or state action when the user requested that change. A request to “connect,” “inspect,” “diagnose,” or “prepare” does not authorize generator output ON.

## 2. Connection discipline

- Use one control process per instrument where practical.
- Serialize query/send-read pairs; SCPI responses have no request IDs.
- Reconnect after discovery when the selected device was already connected.
- Do not reuse a failed or closed socket.
- Treat explicit IPs as strict targets.
- Do not start a new scan automatically after a control connection fails.
- Require manual selection after every actual scan, even with one candidate.

Endpoints are installation-specific and are not permanent identities:

| Instrument | Current/default endpoint | Stable identity |
| --- | --- | --- |
| SIGLENT generator | `<generator-ip>:5025` | SIGLENT SDG model + serial from `*IDN?` |
| RIGOL scope | `<dho-ip>:5555` | RIGOL DHO model + serial from `*IDN?` |
| RIGOL MHO scope | `<mho-ip>:5555` | RIGOL MHO model + serial from `*IDN?` |

## 3. Read-only baseline

From the bundled `scripts/` directory, or from a controller directory explicitly supplied by the user:

```powershell
python sdg2122x_control.py --host <generator-ip> idn
python sdg2122x_control.py --host <generator-ip> status
python dho924_control.py --host <dho-ip> idn
python dho924_control.py --host <dho-ip> status
python dho924_control.py --host <mho-ip> idn
python dho924_control.py --host <mho-ip> status
python dho924_control.py --host <mho-ip> afg --channel 1
```

If an address is stale, omit `--host` or explicitly run `discover`. Do not automate selection of a scan result.

## 4. Coordinated generator/scope workflow

For a live signal such as CH1 sine, 10 kHz, 1 Vpp:

1. Read both identities and states.
2. Turn generator CH1 OFF if changing its settings is authorized.
3. Configure generator CH1 without `--enable`.
4. Read generator CH1 back.
5. Configure the scope only as requested. Check channel probe ratio and termination.
6. Enable generator CH1 only when explicitly authorized and the connection is known safe.
7. Measure scope frequency, Vpp, mean/offset, and waveform shape.
8. Compare requested and measured values.
9. Turn output OFF after a temporary test if requested or agreed.

Safe generator CLI sequence:

```powershell
python sdg2122x_control.py output --channel 1 off
python sdg2122x_control.py sine --channel 1 --frequency 10k --amplitude 1 --offset 0 --phase 0
python sdg2122x_control.py status --channel 1
python sdg2122x_control.py output --channel 1 on
python sdg2122x_control.py status --channel 1
```

Scope inspection examples:

```powershell
python dho924_control.py status --channel 1
python dho924_control.py measure VPP --source CHAN1
python dho924_control.py measure FREQUENCY --source CHAN1
```

## 5. Measurement mismatch diagnosis

Check in this order:

1. Vpp versus Vrms versus dBm.
2. Generator `LOAD` setting versus physical load.
3. Scope probe ratio versus physical probe switch.
4. Scope 50-ohm termination or external terminator.
5. Channel vertical scale and clipping.
6. DC offset and coupling.
7. Bandwidth limit and sample rate.
8. Trigger stability.
9. Cable/splitter attenuation and source impedance.

For a sine wave, `Vrms = Vpp / (2 * sqrt(2))` only when offset is excluded and the waveform is not clipped.

## 6. Failure handling

- On timeout, verify port, endpoint, identity, and exclusive access.
- On a write with ambiguous effect, read back the parameter and query the error queue.
- On generator firmware `-108,"Parameter not allowed"`, do not assume failure or success solely from the error; compare readback with the requested value.
- On DHO924 waveform-preview `-200,"Command execute failed"`, query `:WAVeform:SOURce?` and CH1-CH4 display states. Do not write a disabled analog channel as the waveform source; use the structured controller behavior documented in `dho924.md`.
- On scope RAW read failure, verify STOP state, source, point range, storage depth, and timeout.
- On MHO934, do not query `:DVM:CURRent?` while DVM is OFF or `:LA:ACTive?` while LA is OFF; early firmware may not return a text response.
- On a non-target identity, stop target-specific commands and use only generic SCPI.
- On a failed reconnect after scanning, report the failure and wait for an explicit new scan.

## 7. Code-change validation

Use simulator tests for write paths:

```powershell
$env:PYTHONDONTWRITEBYTECODE='1'
python -m unittest -v test_sdg2122x_control.py
python -m unittest -v test_dho924_control.py
```

Use real devices only for read-only validation unless the user explicitly authorizes the changed behavior. Preserve terminal-only operation and the manual-selection scan rule.
