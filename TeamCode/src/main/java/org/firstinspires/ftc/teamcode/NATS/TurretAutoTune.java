package org.firstinspires.ftc.teamcode.NATS;

import com.aaravdhawan25.pidautotuner.PIDFController;
import com.aaravdhawan25.pidautotuner.PIDGains;
import com.aaravdhawan25.pidautotuner.RelayAutoTuner;
import com.aaravdhawan25.pidautotuner.RelayTuningResult;
import com.aaravdhawan25.pidautotuner.ZieglerNicholsCalculator;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * Turret position tuner, built on FTC-AutoTune v1.4.3 (https://ftcautotune.dev).
 *
 * All settings live in RobotConfig (Dashboard: "RobotConfig", section 11), in degrees from where the
 * turret sits when you press START:
 *
 *  1. Direction probe: 0.25 s of +0.5 power, to learn which way the encoder counts (so the relay can
 *     push both ways without you having to guess).
 *  2. Relay test: RelayAutoTuner swings the turret around TURRET_TARGET_OFFSET_DEG until it has
 *     TURRET_CYCLES_TO_COLLECT clean cycles; Ku / Tu come out of it.
  * 3. Results: six candidates from ZieglerNicholsCalculator. Hold A to live-test one, D-pad up/down to
 *     pick another, B to record the tuned kI / kD and the encoder sign.
 *
 * Put the turret in the MIDDLE of its travel first: the test moves TURRET_TARGET_OFFSET_DEG one way
 * and oscillates around that point. TURRET_MAX_EXCURSION_DEG is a hard stop on how far it may move.
 * B aborts. Anything written with B is live only until the robot restarts.
 *
 * The relay measures POWER PER TICK, which is a plain power loop. The tracking turret does not use one:
 * it runs the hub's stock RUN_TO_POSITION cascade, where TURRET_STOCK_kP is a VELOCITY gain (ticks/s of
 * speed goal per tick of error) and TURRET_VELOCITY_kF is the feedforward that makes it full speed. The
 * two are different units, so this tuner only records its numbers: set TURRET_STOCK_kP and
 * TURRET_VELOCITY_kF in the Dashboard (section 2) by watching the turret move.
 */
@TeleOp(name = "AutoTune: Turret", group = "Tuning")
public class TurretAutoTune extends LinearOpMode {

    private DcMotorEx motor;

    /** +1 when positive power raises the encoder count, -1 when it lowers it. */
    private int direction = 1;

    private long startTicks;
    private String failure;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, RobotConfig.TURRET_MOTOR);
        motor.setDirection(RobotConfig.TURRET_MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        telemetry.addLine("FTC-AutoTune v1.4.3: turret position PID");
        telemetry.addData("offset", "%.0f deg", RobotConfig.TUNE_TURRET_TARGET_OFFSET_DEG);
        telemetry.addData("relay", "+/- %.2f power, %.1f deg deadband",
                RobotConfig.TUNE_TURRET_RELAY_AMPLITUDE, RobotConfig.TUNE_TURRET_HYSTERESIS_DEG);
        telemetry.addData("candidates", RobotConfig.TUNE_TURRET_TUNE_INTEGRAL ? "PID family" : "PD family (kI = 0)");
        telemetry.addLine("Centre the turret in its travel. START to tune, B to abort.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        startTicks = motor.getCurrentPosition();

        if (!probeDirection() || !relayTest()) {
            showFailure();
            return;
        }
        if (isStopRequested()) return;
        resultsLoop();
    }

    private double ticksPerDegree() {
        return RobotConfig.TURRET_TICKS_PER_REV / 360.0;
    }

    private double maxPower() {
        return Math.min(1.0, Math.abs(RobotConfig.TURRET_MAX_POWER));
    }

    /** Tick value the relay oscillates around. */
    private long targetTicks() {
        return startTicks + Math.round(RobotConfig.TUNE_TURRET_TARGET_OFFSET_DEG * ticksPerDegree());
    }

    /** Degrees moved since START, in the encoder's own direction. */
    private double degreesFromStart(long ticks) {
        return (ticks - startTicks) / ticksPerDegree();
    }

    private boolean withinExcursion(long ticks) {
        return Math.abs(degreesFromStart(ticks)) <= Math.abs(RobotConfig.TUNE_TURRET_MAX_EXCURSION_DEG);
    }

    // ---------------------------------------------------------------------
    // Direction probe
    // ---------------------------------------------------------------------

    /** Power of the direction probe: high enough to break the turret free of friction and its gearing. */

    /** A short pulse at +RobotConfig.TUNE_TURRET_PROBE_POWER: does the encoder count go up or down? */
    private boolean probeDirection() {
        long before = motor.getCurrentPosition();
        ElapsedTime clock = new ElapsedTime();

        while (opModeIsActive() && clock.seconds() < 0.25) {
            motor.setPower(RobotConfig.TUNE_TURRET_PROBE_POWER);
        }
        motor.setPower(0);
        sleep(200);

        long delta = motor.getCurrentPosition() - before;
        if (Math.abs(delta) < 0.5 * ticksPerDegree()) {
            failure = String.format("The turret barely moved at %.2f power. Check the motor and encoder, "
                    + "or move the turret off a hard stop.", RobotConfig.TUNE_TURRET_PROBE_POWER);
            return false;
        }
        direction = delta > 0 ? 1 : -1;
        return true;
    }

    // ---------------------------------------------------------------------
    // Relay test (Ku / Tu)
    // ---------------------------------------------------------------------

    private boolean relayTest() {
        double amplitude = Math.min(RobotConfig.TUNE_TURRET_RELAY_AMPLITUDE, maxPower());
        double hysteresisTicks = Math.abs(RobotConfig.TUNE_TURRET_HYSTERESIS_DEG) * ticksPerDegree();
        long target = targetTicks();

        RelayAutoTuner tuner = new RelayAutoTuner(target, amplitude, hysteresisTicks,
                RobotConfig.TUNE_TURRET_CYCLES_TO_COLLECT, RobotConfig.TUNE_TURRET_CYCLES_TO_IGNORE);

        ElapsedTime clock = new ElapsedTime();
        int totalCycles = RobotConfig.TUNE_TURRET_CYCLES_TO_COLLECT + RobotConfig.TUNE_TURRET_CYCLES_TO_IGNORE;

        while (opModeIsActive() && !tuner.isFinished()) {
            long ticks = motor.getCurrentPosition();

            if (gamepad1.b) {
                failure = "B was pressed.";
                break;
            }
            if (!withinExcursion(ticks)) {
                failure = String.format("Stopped: the turret moved more than TURRET_MAX_EXCURSION_DEG (%.0f).",
                        RobotConfig.TUNE_TURRET_MAX_EXCURSION_DEG);
                break;
            }

            // Only the output is flipped, so the relay's "high output raises the measurement" rule holds.
            double output = direction * tuner.update(ticks, clock.seconds());
            motor.setPower(Range.clip(output, -maxPower(), maxPower()));

            telemetry.addLine("Relay test (Ziegler-Nichols)");
            telemetry.addData("moved from start", "%.1f deg", degreesFromStart(ticks));
            telemetry.addData("target from start", "%.1f deg", RobotConfig.TUNE_TURRET_TARGET_OFFSET_DEG);
            telemetry.addData("cycles", "%d / %d", tuner.cyclesCollected(), totalCycles);
            telemetry.addData("power", "%.2f", output);
            telemetry.update();

            if (clock.seconds() > RobotConfig.TUNE_TURRET_TIMEOUT_S) {
                failure = "TIMED OUT: no stable oscillation. Raise TURRET_RELAY_AMPLITUDE (try 0.45) "
                        + "or check that the turret turns freely.";
                break;
            }
        }
        motor.setPower(0);

        if (failure != null) return false;

        relayResult = tuner.computeResult();
        if (relayResult == null) {
            failure = "The relay test saw no usable oscillation amplitude. Raise TURRET_RELAY_AMPLITUDE.";
            return false;
        }
        candidates = ZieglerNicholsCalculator.computeCandidates(relayResult, 0.0, RobotConfig.TUNE_TURRET_TUNE_INTEGRAL);
        if (candidates == null || candidates.isEmpty()) {
            failure = "ZieglerNicholsCalculator returned no candidates.";
            return false;
        }
        return true;
    }

    private RelayTuningResult relayResult;
    private List<PIDGains> candidates;

    // ---------------------------------------------------------------------
    // Results: pick a candidate, live-test it, apply it
    // ---------------------------------------------------------------------

    private void resultsLoop() {
        int selected = Math.max(0, Math.min(candidates.size() - 1, RobotConfig.TUNE_TURRET_LIVE_CANDIDATE));
        long target = targetTicks();
        PIDFController live = null;
        int liveIndex = -1;
        boolean previousUp = false;
        boolean previousDown = false;
        boolean previousB = false;
        String applied = "none";

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            if (up && !previousUp) selected = Math.max(0, selected - 1);
            if (down && !previousDown) selected = Math.min(candidates.size() - 1, selected + 1);
            previousUp = up;
            previousDown = down;

            PIDGains gains = candidates.get(selected);

            if (gamepad1.a) {
                if (live == null || liveIndex != selected) {
                    live = PIDFController.fromGains(gains);
                    live.setOutputBounds(-maxPower(), maxPower());
                    liveIndex = selected;
                }
                // kF is 0 for position, so the library's output is pure PID: scale it by the direction
                // the encoder counts in, then clamp to the turret's power limit.
                double power = direction * live.calculate(motor.getCurrentPosition(), target);
                motor.setPower(Range.clip(power, -maxPower(), maxPower()));
            } else {
                live = null;
                liveIndex = -1;
                motor.setPower(0);
            }

            boolean b = gamepad1.b;
            if (b && !previousB) {
                // The relay measures power per tick; the stock RUN_TO_POSITION P is a velocity gain
                // (ticks/s of speed goal per tick), so the kP cannot be copied across. Record the
                // I / D candidates and the encoder sign; tune TURRET_STOCK_kP + TURRET_VELOCITY_kF by hand.
                RobotConfig.TURRET_STOCK_kI = gains.kI;
                RobotConfig.TURRET_STOCK_kD = gains.kD;
                // Positive power must raise the angle Turret reports, or its target would be backwards.
                RobotConfig.TURRET_ENCODER_SIGN = direction;
                applied = gains.label + " (recorded; set TURRET_STOCK_kP yourself)";
            }
            previousB = b;

            long ticks = motor.getCurrentPosition();
            telemetry.addLine("FTC-AutoTune: turret position");
            telemetry.addData("Ku / Tu", "%.5f / %.4f s", relayResult.Ku, relayResult.Tu);
            telemetry.addData("oscillation", "%.1f deg", relayResult.amplitude / ticksPerDegree());
            for (int i = 0; i < candidates.size(); i++) {
                telemetry.addLine((i == selected ? "> " : "  ") + candidates.get(i));
            }
            telemetry.addData("moved / error", "%.1f / %.1f deg",
                    degreesFromStart(ticks), (target - ticks) / ticksPerDegree());
            telemetry.addData("applied to Turret", applied);
            telemetry.addLine("D-pad up/down: pick  A: hold = live test  B: apply");
            telemetry.update();
        }
        motor.setPower(0);
    }

    private void showFailure() {
        motor.setPower(0);
        while (opModeIsActive()) {
            telemetry.addLine("TUNING STOPPED");
            telemetry.addLine(failure == null ? "The OpMode was stopped." : failure);
            telemetry.update();
        }
    }
}
