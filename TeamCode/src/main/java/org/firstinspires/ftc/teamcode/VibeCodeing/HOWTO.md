# FTC Robot Code How-To (BIOBUZZ 2026-27)

Turret, flywheel, intake, transfer, servos, TeleOp and autos, with live tuning in FTC Dashboard. Two gamepads: gamepad 1 drives and shoots, gamepad 2 runs the turret. The code has not been run on a robot by the person who wrote this guide, so check each section on the robot before trusting it.

## 1. What the target is

In BIOBUZZ you score by launching into the upward-facing CELL of your own alliance's HIVE. Each HIVE has two cells, so the turret tracks one of four points: RED CELL 1 / CELL 2 or BLUE CELL 1 / CELL 2. Their field positions (Pedro inches) are set in `Turret.java` (`RED_CELL1_X/Y` ... `BLUE_CELL2_X/Y`) and are editable live in Dashboard. Measure them if you can; the defaults are estimates.

## 2. Files (`VibeCodeing` folder)

| File | What it does |
|---|---|
| `MainTeleOp.java` | Driving, intake, transfer, servos, shooter RPM, regression recording |
| `Turret.java` | Turret control and tracking, drive, RED/BLUE and CELL selection, aim trim |
| `Shooter.java` | Flywheel velocity PIDF (feedforward + P + I) |
| `ShooterRegression.java` | Distance to RPM table and curve fit, live-tunable |
| `AutoActions.java` | Shared actions for autos (track, spin up, shoot, intake, servos) |
| `ShooterAutoTune.java`, `RelayAutoTuner.java` | Optional autotune OpMode for the flywheel |
| `Autos/TwoTip.java`, `Autos/TwoTipExample.java` | Example autos |

Hardware names: `shooter, transfer, turret, intake, Fl, Fr, Bl, Br, flower, stop`. Intake and transfer are set to REVERSE in code.

## 3. First run checklist

1. Point the turret straight ahead, then press INIT. The turret encoder is zeroed at INIT, so this sets 0 degrees.
2. Check the flywheel spins the right way (`Shooter.REVERSED`) and the turret moves the right way when tracking (`Turret.ENCODER_SIGN`).
3. Set the start pose (`Turret.START_X/Y/HEADING`). Auto-aim is only as good as the pose.
4. Check the intake, transfer, flower and stop servos each move as expected.
5. Only then raise RPM and test feeding.

## 4. Controls

**Gamepad 1**

| Input | Action |
|---|---|
| Left stick / right stick X | Drive / turn |
| Right trigger / left trigger | Intake in / out (the transfer follows the intake) |
| A (hold) | Intake + transfer together, stop servo open |
| Right bumper (hold) | Feed to the shooter (blocked until the flywheel is at speed) |
| Left bumper (hold) | Transfer out |
| D-pad up / down | Shooter RPM +/- `RPM_STEP`. With auto RPM on this is a trim added to the fit. |
| D-pad left | Flower servo up/down |
| BACK | Swap RED / BLUE |
| Right stick button (R3) | Swap CELL 1 / CELL 2 |
| Left stick button (L3) | Record current distance + RPM into the regression table |

The shooter is always spinning (there is no on/off button).

**Gamepad 2**

| Input | Action |
|---|---|
| A / B | Turret tracking on / off (starts OFF) |
| D-pad left / right (hold) | Aim trim, counter-clockwise / clockwise |
| D-pad down | Reset the aim trim to 0 |

## 5. Tuning in FTC Dashboard

Connect to the Control Hub Wi-Fi and open http://192.168.43.1:8080/dash. Add the target and actual values to a graph, change a value and watch. Dashboard edits are lost when the OpMode restarts, so copy final numbers into the code.

### Shot table / regression (the main thing to tune)

Open **ShooterRegression** in the Dashboard config. It has an 8-row shot table: `D1`/`R1` ... `D8`/`R8` (distance in inches, flywheel RPM). A row with distance 0 is empty. The first four rows hold the measured shots (31.6 in 2900, 46.1 in 3200, 54.5 in 3200, 66.8 in 3300).

1. Shoot from a spot. If the shot was short or long, change that row's RPM. It applies instantly, no restart.
2. To add a new spot: drive there, adjust RPM with D-pad up/down until it scores reliably, then press **L3**. The distance and RPM go into the next empty row, where you can still edit them. If all 8 rows are full, it replaces the nearest row.
3. If every shot is off by about the same amount, change `SHOT_OFFSET_RPM` instead of editing every row.
4. Choose how the rows become a curve with `FIT_TYPE`:
   - `0` polynomial least-squares fit (degree from `REGRESSION_DEGREE`, 1 = line, 2 = curve). Default.
   - `1` connect the dots: straight lines between your rows, so every row is hit exactly. Easiest for tuning. Outside the first/last row it holds that row's RPM.
   - `2` logarithmic fit, flattens at long range.
   - `3` power-law fit.
5. Telemetry shows the fit's R2 and the table as `distance:rpm` pairs. A fit with low R2 is not following your points; use `FIT_TYPE` 1 or add more rows.
6. `MIN_AUTO_RPM` and `MAX_AUTO_RPM` (in MainTeleOp and AutoActions) clamp the result. The flywheel tops out near 3300 RPM.
7. **Make it permanent:** copy the rows into the `D`/`R` fields and the `DEFAULT_DIST`/`DEFAULT_RPM` arrays in `ShooterRegression.java`. Auto and TeleOp share the same table.

Tips: shoot from several distances (short, mid, long); more rows beat a fancier curve; keep the battery charged when recording, since a weak battery changes the RPM that scores. `AUTO_RPM_FROM_DISTANCE` in MainTeleOp turns the distance-based RPM on or off.

### Turret aim

- `Turret.kP` (P only), `MAX_POWER` and `TOLERANCE` set how it moves. Raise `kP` until it is quick without oscillating.
- **Aim trim:** if shots land consistently left or right, hold gamepad 2 D-pad left/right to nudge the aim while tracking. `TRIM_RATE_DEG_PER_SEC` and `TRIM_MAX_DEG` set the speed and limit; D-pad down resets. If the trim you settle on is always the same, fix the cause instead: check `OFFSET_X/OFFSET_Y` (turret pivot offset), the cell coordinates and the start pose, then reset the trim.
- Limits are -315 to +45 degrees with a 5 degree soft margin; the target wraps into that range.

### Flywheel

- `Shooter.kV` and `kS` set the base power for a target RPM, `kP`/`kI` correct the error. Set `kP` and `kI` to 0, adjust `kV`/`kS` until the RPM sits near target, then add `kP` and a small `kI`.
- If the RPM sags after each ball, raise `kP`/`kI` a little. `AT_SPEED_TOLERANCE_RPM` and `AT_SPEED_HOLD_SEC` decide when feeding is allowed.
- The optional `ShooterAutoTune` OpMode can find starting gains.

### Servos

`AutoActions.FLOWER_UP/FLOWER_DOWN` and `STOP_DOWN/STOP_BLOCKING` are the servo positions, shared by TeleOp and autos.

## 6. Troubleshooting

| Problem | Fix |
|---|---|
| Turret does nothing | Tracking starts OFF. Press gamepad 2 A. |
| Turret runs to a limit | Check `ENCODER_SIGN`, and that the turret was straight ahead at INIT. |
| Shots miss in a consistent direction | Aim trim (gamepad 2 D-pad), then check pose, offsets and cell coordinates. |
| Shots consistently short/long at one distance | Edit that row's RPM in ShooterRegression. |
| Shots short/long everywhere | `SHOT_OFFSET_RPM`. |
| Transfer does not feed | Flywheel not at speed. Wait, or untick `MainTeleOp.REQUIRE_AT_SPEED`. |
| Flywheel RPM oscillates | Lower `kP`, or raise `VELOCITY_FILTER` a little. |
| Auto starts with the wrong aim | TeleOp starts at `START_X/Y/HEADING`, not where the auto ended. Set the start pose to match, or carry the pose over. |
| Build error on Pedro imports | This code uses Pedro 3 (`follower.pose()`, `follower.manual(...)`). |

## 7. Not verified

- Cell coordinates are estimates, not measured.
- Nothing here has been proven on a competition field.