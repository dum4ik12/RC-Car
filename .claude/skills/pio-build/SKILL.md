---
name: pio-build
description: Build, flash, monitor, clean or unit-test the RC_CAR ESP32 firmware with PlatformIO, and diagnose compile, link, upload or boot/crash errors. Use after any code change that must be compiled, when the user asks to upload/flash/monitor, or when a build or the board fails.
argument-hint: "[build|upload|monitor|clean|test|size]"
---

# PlatformIO build / flash / monitor for RC_CAR

`pio` is **not on PATH** in Claude's shells. Always use the full path:

```powershell
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"
```
(Bash tool: `~/.platformio/penv/Scripts/pio.exe`)

Requested action: `$ARGUMENTS` (default: build).

## Build (always safe; do it after every change)
```powershell
$out = & $pio run 2>&1 | Out-String -Width 400
($out -split "`n") | Where-Object { $_ -match 'warning|error|RAM:|Flash:|SUCCESS|FAILED' }
```
- The result must be `SUCCESS` **with zero warnings** in `src/` and `lib/Actuators`. Fix warnings; don't silence them.
- Check a file's real compile flags with `& $pio run -v`, after deleting that `.o` from `.pio\build\esp32dev\...`.
- To inspect size: `& $pio run -t size`. RAM and Flash percentages are also printed in the build summary.

## Upload: ASK FIRST
Flashing starts the firmware immediately, and **the motor can spin**. Before uploading:
1. Get the user's explicit OK, and remind them: wheels off the ground.
2. Run `& $pio run -t upload`. Add `--upload-port COMx` if there are several ports; list them with `& $pio device list`.

## Monitor
`pio device monitor` runs forever. Don't run it in the foreground.
- The simplest option is to ask the user to open the PlatformIO Serial Monitor in VS Code.
- To capture logs yourself, run `& $pio device monitor` with `run_in_background: true`, then read its output and stop it when done.
- The monitor is 115200 baud with the `esp32_exception_decoder` filter, which turns backtraces into file:line.

## Clean
`& $pio run -t clean` is for when you've renamed or moved libraries, when you get odd linker errors, or when you need a full rebuild.

## Unit tests (pure logic only: `lib/CrsfProtocol`, `lib/Control`)
Hardware-free code lives in its own lib with no `driver/*` includes and is tested on the host. `platformio.ini` already has the env (`[env:native]`, Unity, `lib_ignore = Actuators, Radio`) and `default_envs = esp32dev`, so `pio run` never touches it. The tests are `test/test_crsf/` and `test/test_control/`.

Run with `& $pio test -e native`. **Always pass `-e native`** (`esp32dev` has `test_ignore = test_*` as a guard). On Windows the native platform needs a host GCC on PATH (MSYS2: `pacman -S mingw-w64-ucrt-x86_64-gcc`, then add `C:\msys64\ucrt64\bin` to PATH). If it is missing, say so; the firmware build still runs the `static_assert` self-tests (`crsf/SelfTest.h`, `DriveControllerSelfTest.h`), which cover the same core scenarios.

## Troubleshooting
| Symptom | Likely cause / fix |
|---|---|
| `Failed to connect ... Wrong boot mode` / `Timed out waiting for packet header` | Hold **BOOT** while the upload starts. GPIO2 (L_EN) must be LOW at reset, so check its 10 kΩ pull-down. Close any open serial monitor. |
| `could not open port` / access denied | Another program (a monitor) holds the COM port. Close it. |
| `Brownout detector was triggered` / random resets when the motor starts | Motor current spikes are sagging the supply. Power the ESP32 from a separate BEC or regulator, add a bulk capacitor near the BTS7960, and use a common GND. |
| `Guru Meditation Error` + backtrace | Read the decoded file:line from the monitor. Usually a null pointer, a stack overflow or an ISR problem. |
| `***ERROR*** A stack overflow in task X` | Raise that task's stack (bytes on ESP-IDF). See the `freertos-tasks` skill. |
| `Task watchdog got triggered` | A task busy-looped without blocking. Add a blocking wait. Never spin. |
| Undefined reference to a `lib/` symbol | The library isn't included from `src/`, so the LDF didn't pick it up. `#include` its header in the app, or clean. |
| LEDC `ledc_timer_config failed` | freq × 2^resolution > 80 MHz, or the timer/channel is already used with other settings. See the `actuator-driver` skill. |
