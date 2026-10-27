# FTC Robot Code How-To

A start-to-finish guide for the turret, flywheel, intake, transfer and TeleOp code, including tuning with FTC Dashboard and the FtcAutoTune library. Everything is driven from ONE gamepad (gamepad 1). The code has not been compiled or run on a robot, so expect to fix small build errors on the first try.

## 1. BIOBUZZ has no goals: the target is the HIVE

The 2026-27 game is BIOBUZZ. Per the competition manual, you score by launching elements into the upward-facing CELL of your own alliance's HIVE, the structure in the middle of the field. Each HIVE TIP is worth 20 points. So "red goal" in this code means a CELL on the RED HIVE, and each HIVE has two cells you can switch between.

The manual gives CELL openings of about 20 in wide, 14 in tall and 12 in deep, and says each HIVE pivots about 44 in above the tiles. It does not give field (x, y) coordinates, so you have to measure them (section 9). Until you do, auto-aim stays blocked and the turret stays on manual aim.

## 2. Files

Put all six files in the SAME folder. They are set up for `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/VibeCodeing/` (the first line of each file is `package org.firstinspires.ftc.teamcode.VibeCodeing;`). If you use a different folder, change that first line in all six files to match the folder.

- `Turret.java`: position PID in degrees (435 RPM Yellow Jacket, 20T to 80T (4:1), OPTI V2 encoder at 4096 counts per revolution on the pinion shaft, so about 45.5 ticks per degree). Limits of 180 degrees each side (-180° to +180°); the target wraps into that range, and the turret goes the long way round when the target crosses the back, because the cables stop it passing 180.
- `Shooter.java`: velocity PIDF in flywheel RPM (28 ticks per motor rev, 10T to 18T, max about 3333 RPM).
- `ShooterRegression.java`: distance to flywheel RPM regression (least-squares line or curve). Needed by `MainTeleOp.java`, so copy it too (that makes seven files in total).
- `RelayAutoTuner.java`: the built-in relay-feedback tuner used by the two tuning OpModes.
- `TurretAutoTune.java` and `ShooterAutoTune.java`: tuning OpModes, listed under "Tuning" on the Driver Station.
- `MainTeleOp.java`: driving, mechanisms, turret aiming, the RED/BLUE swap and the CELL 1/2 switch.

If the build reports a name clash with the FtcAutoTune library's own `RelayAutoTuner`, rename the one in `RelayAutoTuner.java` (and its uses) to something like `BuiltInRelayTuner`.

## 3. Hardware configuration names

- `Fl`, `Fr`, `Bl`, `Br`: the four mecanum drive motors. These must match the names in your Pedro Constants.
- `shooter`: bare 5000 series motor, encoder cable plugged in.
- `turret`: 435 RPM Yellow Jacket, encoder cable plugged in.
- `intake` and `transfer`: separate motors, both set to REVERSE in code.

## 4. Install

1. Confirm FTC Dashboard is a dependency (the Pedro quickstart normally includes it).
2. Copy the six files into TeamCode.
3. `MainTeleOp.java` is written for Pedro 3. It imports `Constants` from `org.firstinspires.ftc.teamcode.pedro` and calls `Constants.create(hardwareMap)`, as in the Pedro 3 docs. Change that import and call to match where your own Constants class lives and what its factory method is called.
4. Replace the placeholder start pose in `MainTeleOp.init()` (currently 72, 72, 0, with heading in radians) with your real TeleOp start pose. Check that your Pedro 3 field coordinates use the same 0 to 144 origin the target values assume (section 9).
5. Build and deploy.
6. Connect to the Control Hub Wi-Fi and open http://192.168.43.1:8080/dash for FTC Dashboard.

## 5. First run checklist

1. Point the turret straight ahead, then press INIT. The turret encoder is zeroed at INIT, so this sets 0 degrees.
2. Shooter direction: in Dashboard set `Shooter.TARGET_RPM` to about 500, press A, and check the flywheel spins the correct way. If not, tick `Shooter.REVERSED` and re-run INIT. Press B to stop.
3. Turret direction: hold D-pad right. The turret should turn clockwise seen from above (positive turret angle is counter-clockwise, the same as Pedro heading). If it goes the wrong way, tick `Turret.REVERSED` and re-run INIT.
4. Intake and transfer: right trigger should pull pieces in, right bumper should feed toward the shooter. Both motors are already set to REVERSE.
5. Only then raise the shooter RPM and test feeding.

## 6. Controls (one gamepad)

| Input | Action |
|---|---|
| Left stick | Drive (move) |
| Right stick X | Turn |
| Right trigger / left trigger | Intake in / out (the transfer runs with the intake) |
| Right bumper (hold) | Transfer in to feed the shooter, blocked until the shooter is at speed |
| Left bumper (hold) | Transfer out |
| A / B | Shooter on / off |
| D-pad up / down | Shooter RPM +100 / -100 |
| D-pad left / right (hold) | Turret manual aim (left = counter-clockwise, right = clockwise). Only works with auto-aim off, and it stops at the -180 / +180 limits instead of wrapping. |
| X | Toggle turret auto-aim (it now starts ON, and `TARGETS_CALIBRATED` is set to true in the code, so press X first if you want manual aim) |
| BACK | Swap target RED / BLUE (starts on RED) |
| Right stick button (R3) | Switch between the alliance's CELL 1 (upper) and CELL 2 (lower) |
| Y | Zero the turret encoder here |
| Left stick button (L3) | Record a regression sample (current distance to the target + current shooter RPM) |

Telemetry shows the current alliance target and cell, turret target vs angle, shooter target vs RPM, and "at speed". Numeric values appear as live graphs in Dashboard.

## 7. Tuning with the built-in tuners

### Turret

1. Point the turret straight ahead and make sure it can move about 30 degrees either way with clear cables.
2. Run "AutoTune: Turret" from the Driver Station and press START. It swings the turret back and forth about 0 degrees for several cycles. Press B to abort.
3. When it finishes, press A for conservative gains (Tyreus-Luyben) or X for classic Ziegler-Nichols (more aggressive).
4. Open the Turret config in Dashboard to see the values live. Copy the final kP, kI and kD into `Turret.java` so they survive a restart.

### Shooter

1. Clear the shooter path.
2. Run "AutoTune: Shooter". Phase 1 runs two open-loop powers and solves kS and kV. Phase 2 runs the relay test around `TUNE_RPM` to get kP and kI.
3. Press A to apply the gains. Copy kP, kI, kV and kS into `Shooter.java`.

### Fine-tuning in Dashboard (both mechanisms)

- Add the target and actual values to a Dashboard graph.
- Turret: raise kP until it moves quickly with a little overshoot, add kD to damp it, and use kI only if it stops short of the target. Use kS if it stalls near the target.
- Flywheel: set kP and kI to 0 and adjust kV until the RPM sits close to the target, then add kP, then a small kI if it still sags. After a ball is fed, the RPM should recover quickly.

## 8. Tuning with the FtcAutoTune library

### What it does

Instead of guessing PID(F) constants, FtcAutoTune uses relay-feedback tuning. It drives the motor back and forth (bang-bang style) to cause a controlled oscillation, and measures the ultimate gain (Ku) and period (Tu). It then applies Ziegler-Nichols rules to produce six candidate gain sets, from gentle to aggressive, and fits a feedforward value (kF) by least squares.

### Setup

1. Add JitPack to the repositories in your `build.dependencies.gradle`:

```groovy
repositories {
    mavenCentral()
    google()
    maven { url = 'https://jitpack.io' }
}
```

2. Add the dependencies (use the latest version; v1.4.3 at the time of the guide):

```groovy
dependencies {
    implementation 'dev.ftcautotune.FtcAutoTune:pidautotuner-core:v1.4.3'
    implementation 'dev.ftcautotune.FtcAutoTune:pidautotuner-ftc:v1.4.3'
}
```

3. Sync Gradle.
4. From the FTC-AutoTune QuickStart repository, copy only the template files into your TeamCode package (not the master files):
   - `TuningConfig.java`: the only file you edit. It holds hardware names, targets and thresholds.
   - `PositionPIDTunerOpMode.java`: for the turret.
   - `VelocityPIDFTunerOpMode.java`: for the flywheel.
   - `DualMotorVelocityPIDFTunerOpMode.java`: only for dual-motor shooters, so you don't need it.
5. Edit `TuningConfig.java`:
   - `MOTOR_NAME`: `turret` or `shooter`, exactly as in your hardware config.
   - `TICKS_PER_REV`: the encoder resolution of that motor. For the turret use 4096 (the OPTI V2, on the pinion shaft). For the flywheel use 28 (the bare 5000 series motor). The Yellow Jacket's own built-in encoder would be 383.6.
   - `RELAY_AMPLITUDE`: about 0.3 for the turret, 0.5 to 0.8 for the flywheel.
   - `TUNE_INTEGRAL_TERM`: keep `false` for the flywheel so integral windup can't build up; feedforward handles steady state.
   - `POSITION_TARGET_TICKS` (turret): a small swing, for example 1400 ticks (about 30 degrees with the OPTI V2 and the current gearing). `VELOCITY_TARGET_TICKS_PER_SEC` (flywheel): see the conversion below.

### Running it

1. A live dashboard is recommended so you can edit values without redeploying. You already have FTC Dashboard; Pulse (by aaravdhawan25) is the other option the guide mentions. The library's own dashboard (per its README) runs at http://192.168.43.1:8001.
2. On the Driver Station, open the FTC-AutoTune group and pick the tuner OpMode.
3. Press INIT to read the startup telemetry, then START.
4. Phase 1 (relay test): the motor oscillates automatically for about 10 to 30 seconds. Keep your hands off the robot.
5. Phase 2 (feedforward sweep): for velocity tuning, it sweeps power levels to compute kF.
6. Read the candidate values in telemetry, hold gamepad 1 A to live-test the classic Ziegler-Nichols candidate, then copy the constants you want.

### Converting the gains for this code

The library works in encoder ticks, but this code uses degrees (turret) and flywheel RPM (shooter), so convert before using its gains:

- Turret: multiply kP, kI and kD by the ticks per degree (about 45.5 with the OPTI V2 and the current 20T to 80T gearing: 4096 x 4 / 360), or call `Turret.applyTickGains(kp, ki, kd)` once in code.
- Shooter: the library measures motor-side ticks per second. Multiply kP, kI and kD by 0.84, or call `Shooter.applyTickGains(kp, ki, kd)`. Target ticks per second for the flywheel = flywheel RPM x 0.84 (2200 RPM is about 1848 ticks per second).
- Feedforward: the library's kF units aren't spelled out in the guide. Check them before converting. If it is power per tick per second, kV is about kF x 0.84. Otherwise use the kV from the built-in shooter tuner.

## 9. Setting the RED and BLUE targets

Each alliance's HIVE has two cells. In Dashboard, open `MainTeleOp` and set `RED_CELL1_X/Y`, `RED_CELL2_X/Y`, `BLUE_CELL1_X/Y` and `BLUE_CELL2_X/Y` to the field position of each cell opening (CELL 1 is the upper one, CELL 2 the lower one), in Pedro field inches (the field is 144 by 144). Then tick `TARGETS_CALIBRATED`.

- Measure each cell opening from the field's origin corner along both axes with a tape measure, or read it off the field figures in the competition manual (Section 9, ARENA). Check your Pedro coordinate orientation first, since the origin and axes depend on your Pedro setup.
- The default values (red: 58, 85 and 58, 59; blue: 86, 85 and 86, 59) are estimates read off a field image, roughly 3 in accuracy, and auto-aim will not engage while `TARGETS_CALIBRATED` is off.
- `TURRET_ZERO_OFFSET_DEG` should be 0 if turret 0 degrees points straight along the robot's heading.
- Auto-aim is only as good as your Pedro pose, so set a correct start pose.
- Press BACK at any time to swap between RED and BLUE. It starts on RED. Press the right stick button (R3) to switch between that alliance's CELL 1 (upper) and CELL 2 (lower); it starts on CELL 1.

## 9b. Shooter distance regression

The shooter RPM can follow the distance from the robot to the selected HIVE cell, using a least-squares fit through your own measured shots.

1. Make sure auto-aim targets and the start pose are right (section 9), because the distance comes from the Pedro pose to the selected cell.
2. The built-in table in `ShooterRegression.java` now holds the shots measured on 26 Oct 2026 (2900 / 3200 / 3200 / 3300 RPM at 31.6 / 46.1 / 54.5 / 66.8 in), and `AUTO_RPM_FROM_DISTANCE` starts on. Untick it in Dashboard while you add or re-measure samples.
3. Drive to a spot, press A to spin up, and adjust the RPM with D-pad up and down until shots score reliably from there. (With `AUTO_RPM_FROM_DISTANCE` on, D-pad up and down is a trim on top of the fit.)
4. Press the left stick button (L3). This records the current distance and RPM as one sample.
5. Repeat from at least 4 or 5 different distances (close to far). Once 2 or more samples are recorded they replace the placeholder table, and telemetry shows the fit, its R2 quality number, and the list of samples as `distance:rpm` pairs.
6. Tick `AUTO_RPM_FROM_DISTANCE` in Dashboard. The RPM now follows the fit as you drive, and D-pad up and down becomes a trim you add on top. Use `REGRESSION_DEGREE` 1 for a straight line or 2 for a curve. `MIN_AUTO_RPM` and `MAX_AUTO_RPM` clamp the result (the flywheel tops out around 3333 RPM).
7. Recorded samples are lost when the OpMode stops. To keep them, copy the `samples` line from telemetry into the `DEFAULT_DIST` and `DEFAULT_RPM` arrays in `ShooterRegression.java`.

Notes: distances are in field inches; a higher R2 (close to 1) means a tighter fit; with only two samples the fit is a straight line whatever the degree setting. The fit is only as good as the shot data and the pose accuracy.

## 9c. Autonomous helpers (`AutoActions`)

`AutoActions.java` drives the same turret, shooter, intake and transfer for auto, without changing any TeleOp behaviour (same hardware names, same PID gains, same measured RPM table). Every call is non-blocking; `update()` must run every loop so both PIDs keep going.

```java
follower = Constants.create(hardwareMap);
follower.setPose(start);
follower.update();
AutoActions mech = new AutoActions(hardwareMap, follower); // turret must point straight ahead here
mech.spinUpForDistance();                                 // or mech.spinUp(3200)

while (opModeIsActive()) {
    follower.update();
    mech.update();                        // turret tracks + flywheel holds RPM
    if (mech.isReadyToShoot()) mech.shoot();
    mech.intakeIn(); mech.intakeStop();   // intake
    mech.transferIn(); mech.transferStop();
}
mech.stopAll();
```

- `spinUpForDistance()` uses the `ShooterRegression` table from 9b, evaluated at the distance to the tracked goal; `trackGoal(red, cell1)` picks a Turret goal cell, `aimAt(x, y)` tracks any other point.
- `shoot()` feeds only once the flywheel is at speed (`GATE_FEED_ON_SPEED`); `isReadyToShoot()` also needs the turret within `AIM_TOLERANCE_DEG`.
- The turret must be pointing straight ahead at INIT, exactly like TeleOp, because that is where 0 degrees is captured.

## 10. Troubleshooting

| Problem | Fix |
|---|---|
| Turret runs away or the tuner aborts with "moved too far" | Positive power moves it the wrong way. Toggle `Turret.REVERSED` and re-run INIT. |
| Flywheel spins the wrong way | Toggle `Shooter.REVERSED` and re-run INIT. |
| Turret does nothing, or runs to a limit and stops | Open TeleOp telemetry and watch "turret encoder ticks" while you turn the turret by hand. (1) Stuck at 0: the encoder is on a different port, so set `Turret.ENCODER_NAME` to that port's config name. (2) Ticks go DOWN when positive power drives the turret up: tick `Turret.ENCODER_REVERSED` (the PID is working against itself). (3) Angle is off by a fixed factor: fix `TICKS_PER_MOTOR_REV` / `MOTOR_TEETH` / `TURRET_TEETH` (now 20 and 80). |
| Turret angle looks wrong | Check `Turret.TICKS_PER_MOTOR_REV`. It is 4096 for the OPTI V2, assumed to be on the 20T pinion shaft. If the OPTI V2 sits on the turret axis instead, set `MOTOR_TEETH` = `TURRET_TEETH` (ratio 1:1) and leave `TICKS_PER_MOTOR_REV` at 4096. Turn the turret exactly one full rotation by hand to check: the angle should read about 360. |
| Auto-aim does nothing | Set the target coordinates and tick `TARGETS_CALIBRATED`. |
| Transfer does not feed | The shooter is not at speed. Wait for "at speed", or untick `MainTeleOp.REQUIRE_AT_SPEED`. |
| Flywheel RPM oscillates | Lower kP, or raise `Shooter.VELOCITY_FILTER` a little. |
| Shooter tuner fails with "RPM barely changed" | Check the shooter encoder cable and the motor name `shooter`. |
| Build error on Pedro imports | `MainTeleOp` uses the Pedro 3 packages (`com.pedropathing.math.Pose`, `com.pedropathing.follower.ManualDrive`). Pedro 3 renamed things (`follower.pose()`, `follower.setPose(...)`, `follower.manual(...)`), so older Pedro 1.x/2.x code won't match. Check the docs at pedropathing.com for your exact version. |
| Strafe or turn feels reversed | `MainTeleOp` follows the Pedro 3 docs example (lateral = `left_stick_x`, turn = `right_stick_x`). Negate that term in the drive block if your robot goes the wrong way. |
| "package ... does not exist" for Turret, Shooter or Constants | All the files must share one package. If you put them in a subfolder (for example `VibeCodeing`), change the `package` line at the top of every file to match, and fix the `Constants` import. |
| Gradle can't find the FtcAutoTune library | Check the JitPack repository line and use a version tag that exists. |

## 11. Not verified

- Section 8 follows the guide you supplied. I haven't run the library against this code.
- BIOBUZZ target coordinates are not in the manual text; section 9 explains how to supply them.
- Nothing here has been compiled or run on hardware.

Sources: [Competition Manual, Section 9 ARENA](https://ftc-resources.firstinspires.org/ftc/game/manual-09), [Competition Manual, Section 10 Game Details](https://ftc-resources.firstinspires.org/ftc/game/manual-10), [FtcAutoTune on GitHub](https://github.com/aaravdhawan25/FtcAutoTune).
