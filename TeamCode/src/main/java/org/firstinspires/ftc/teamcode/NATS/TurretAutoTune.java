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
 * All settings live in {@link TuningConfig} (Dashboard: "TuningConfig"), in degrees from where the
 * turret sits when you press START:
 *
 *  1. Direction probe: 0.25 s of +0.2 power, to learn which way the encoder counts (so the relay can
 *     push both ways without you having to guess).
 *  2. Relay test: RelayAutoTuner swings the turret around TURRET_TARGET_OFFSET_DEG until it has
 *     TURRET_CYCLES_TO_COLLECT clean cycles; Ku / Tu come out of it.
 *  3. Results: six candidates from ZieglerNicholsCalculator. Hold A to live-test one, D-pad up/down to
 *     pick another, B to write it into Turret (kP, kI, kD).
 *
 * Put the turret in the MIDDLE of its travel first: the test moves TURRET_TARGET_OFFSET_DEG one way
 * and oscillates around that point. TURRET_MAX_EXCURSION_DEG is a hard stop on how far it may move.
 * B aborts. Gains written with B are live only until the robot restarts: copy them into
 * RobotConfig.TURRET_kP / TURRET_kI / TURRET_kD to keep them.
 *
 * The relay tunes in encoder ticks (power per tick). On apply the gains are converted to the radians
 * Turret's own controller uses:  k_rad = k_ticks * TURRET_TICKS_PER_REV / (2 pi).
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
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        telemetry.addLine("FTC-AutoTune v1.4.3: turret position PID");
        telemetry.addData("offset", "%.0f deg", TuningConfig.TURRET_TARGET_OFFSET_DEG);
        telemetry.addData("relay", "+/- %.2f power, %.1f deg deadband",
                TuningConfig.TURRET_RELAY_AMPLITUDE, TuningConfig.TURRET_HYSTERESIS_DEG);
        telemetry.addData("candidates", TuningConfig.TURRET_TUNE_INTEGRAL ? "PID family" : "PD family (kI = 0)");
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

    private double ticksPerRadian() {
        return RobotConfig.TURRET_TICKS_PER_REV / (2.0 * Math.PI);
    }

    private double maxPower() {
        return Math.min(1.0, Math.abs(RobotConfig.TURRET_MAX_POWER));
    }

    /** Tick value the relay oscillates around. */
    private long targetTicks() {
        return startTicks + Math.round(TuningConfig.TURRET_TARGET_OFFSET_DEG * ticksPerDegree());
    }

    /** Degrees moved since START, in the encoder's own direction. */
    private double degreesFromStart(long ticks) {
        return (ticks - startTicks) / ticksPerDegree();
    }

    private boolean withinExcursion(long ticks) {
        return Math.abs(degreesFromStart(ticks)) <= Math.abs(TuningConfig.TURRET_MAX_EXCURSION_DEG);
    }

    // ---------------------------------------------------------------------
    // Direction probe
    // ---------------------------------------------------------------------

    /** A short pulse at +0.2 power: does the encoder count go up or down? */
    private boolean probeDirection() {
        long before = motor.getCurrentPosition();
        ElapsedTime clock = new ElapsedTime();

        while (opModeIsActive() && clock.seconds() < 0.25) {
            motor.setPower(0.2);
        }
        motor.setPower(0);
        sleep(200);

        long delta = motor.getCurrentPosition() - before;
        if (Math.abs(delta) < 0.5 * ticksPerDegree()) {
            failure = "The turret barely moved at 0.2 power. Check the motor and encoder, "
                    + "or move the turret off a hard stop.";
            return false;
        }
        direction = delta > 0 ? 1 : -1;
        return true;
    }

    // ---------------------------------------------------------------------
    // Relay test (Ku / Tu)
    // ---------------------------------------------------------------------

    private boolean relayTest() {
        double amplitude = Math.min(TuningConfig.TURRET_RELAY_AMPLITUDE, maxPower());
        double hysteresisTicks = Math.abs(TuningConfig.TURRET_HYSTERESIS_DEG) * ticksPerDegree();
        long target = targetTicks();

        RelayAutoTuner tuner = new RelayAutoTuner(target, amplitude, hysteresisTicks,
                TuningConfig.TURRET_CYCLES_TO_COLLECT, TuningConfig.TURRET_CYCLES_TO_IGNORE);

        ElapsedTime clock = new ElapsedTime();
        int totalCycles = TuningConfig.TURRET_CYCLES_TO_COLLECT + TuningConfig.TURRET_CYCLES_TO_IGNORE;

        while (opModeIsActive() && !tuner.isFinished()) {
            long ticks = motor.getCurrentPosition();

            if (gamepad1.b) {
                failure = "B was pressed.";
                break;
            }
            if (!withinExcursion(ticks)) {
                failure = String.format("Stopped: the turret moved more than TURRET_MAX_EXCURSION_DEG (%.0f).",
                        TuningConfig.TURRET_MAX_EXCURSION_DEG);
                break;
            }

            // Only the output is flipped, so the relay's "high output raises the measurement" rule holds.
            double output = direction * tuner.update(ticks, clock.seconds());
            motor.setPower(Range.clip(output, -maxPower(), maxPower()));

            telemetry.addLine("Relay test (Ziegler-Nichols)");
            telemetry.addData("moved from start", "%.1f deg", degreesFromStart(ticks));
            telemetry.addData("target from start", "%.1f deg", TuningConfig.TURRET_TARGET_OFFSET_DEG);
            telemetry.addData("cycles", "%d / %d", tuner.cyclesCollected(), totalCycles);
            telemetry.addData("power", "%.2f", output);
            telemetry.update();

            if (clock.seconds() > TuningConfig.TURRET_TIMEOUT_S) {
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
        candidates = ZieglerNicholsCalculator.computeCandidates(relayResult, 0.0, TuningConfig.TURRET_TUNE_INTEGRAL);
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
        int selected = Math.max(0, Math.min(candidates.size() - 1, TuningConfig.TURRET_LIVE_CANDIDATE));
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
                double toRadians = ticksPerRadian();
                Turret.kP = gains.kP * toRadians;
                Turret.kI = gains.kI * toRadians;
                Turret.kD = gains.kD * toRadians;
                // Positive power must raise the angle Turret reports, or its P loop drives the wrong way.
                Turret.ENCODER_SIGN = direction;
                applied = gains.label;
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
