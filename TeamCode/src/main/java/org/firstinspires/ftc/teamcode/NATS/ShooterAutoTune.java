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
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Flywheel velocity tuner, built on FTC-AutoTune v1.4.3 (https://ftcautotune.dev).
 *
 * All settings live in RobotConfig (Dashboard: "RobotConfig", section 11), in flywheel RPM:
 *
 *  Phase 1 - feedforward sweep (open-loop characterization): the flywheel runs at each
 *            SHOOTER_FF_POWERS value until it settles, and the (power, RPM) pairs are fitted as
 *            power = kS + kV * RPM. The library's own fit goes through the origin and cannot see kS,
 *            so this fits the intercept too and hands kV to the library as the candidate kF.
 *  Phase 2 - relay test: RelayAutoTuner swings the power by +/- SHOOTER_RELAY_AMPLITUDE around the
 *            feedforward power for the target RPM, which forces a limit cycle. Ku / Tu come out of it.
 *  Results - ZieglerNicholsCalculator gives six candidates. Hold A to live-test one, D-pad up/down to
 *            pick another, B to write it into RobotConfig.SHOOTER_kP / _kI / _kD / _kV / _kS.
 *
 * Nothing is in the shooter path. B aborts. Gains written with B are live only until the robot is
 * restarted: copy them into RobotConfig (section 4) to keep them.
 */
@TeleOp(name = "AutoTune: Shooter", group = "Tuning")
public class ShooterAutoTune extends LinearOpMode {

    private Shooter shooter;
    private String failure;

    /** Feedforward from the sweep, in the Shooter controller's units: power = kS + kV * RPM. */
    private double kS;
    private double kV;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Shooter();
        shooter.init(hardwareMap);

        telemetry.addLine("FTC-AutoTune v1.4.3: flywheel velocity PIDF");
        telemetry.addData("target", "%.0f RPM", RobotConfig.TUNE_SHOOTER_TARGET_RPM);
        telemetry.addData("relay", "+/- %.2f power", RobotConfig.TUNE_SHOOTER_RELAY_AMPLITUDE);
        telemetry.addData("candidates", RobotConfig.TUNE_SHOOTER_TUNE_INTEGRAL ? "PID family" : "PD family (kI = 0)");
        telemetry.addLine("Clear the shooter path. START to tune, B to abort.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        if (!feedforwardSweep() || !relayTest()) {
            showFailure();
            return;
        }
        shooter.stop();
        if (isStopRequested() || failure != null) {
            showFailure();
            return;
        }
        resultsLoop();
    }

    // ---------------------------------------------------------------------
    // Phase 1: feedforward
    // ---------------------------------------------------------------------

    /** Sweeps the open-loop powers and fits power = kS + kV * RPM. Sets kS / kV, or sets failure. */
    private boolean feedforwardSweep() {
        double[] powers = RobotConfig.TUNE_SHOOTER_FF_POWERS;
        if (powers == null || powers.length < 2) {
            failure = "SHOOTER_FF_POWERS needs at least two power levels.";
            return false;
        }

        double n = 0, sumPower = 0, sumRpm = 0, sumPowerRpm = 0, sumRpm2 = 0;

        for (double power : powers) {
            if (gamepad1.b) {
                failure = "B was pressed.";
                return false;
            }
            double rpm = measureSteadyRpm(power);
            if (Double.isNaN(rpm)) return false;

            // Normalise to the nominal battery voltage, the same way Shooter's feedforward does.
            double nominalPower = power * shooter.getVoltage() / RobotConfig.SHOOTER_NOMINAL_VOLTAGE;
            n++;
            sumPower += nominalPower;
            sumRpm += rpm;
            sumPowerRpm += nominalPower * rpm;
            sumRpm2 += rpm * rpm;
        }

        if (n < 2) {
            failure = "The sweep produced no measurable RPM. Check the encoder and RobotConfig.SHOOTER_ENCODER_REVERSED / SHOOTER_MOTOR_REVERSED.";
            return false;
        }

        double denominator = n * sumRpm2 - sumRpm * sumRpm;
        if (Math.abs(denominator) < 1e-9) {
            failure = "RPM barely changed across the sweep. Check the encoder cable and the motor.";
            return false;
        }
        kV = (n * sumPowerRpm - sumRpm * sumPower) / denominator;
        kS = (sumPower - kV * sumRpm) / n;

        if (kV <= 0) {
            failure = "kV came out <= 0: RPM fell as power rose. Flip RobotConfig.SHOOTER_ENCODER_REVERSED or SHOOTER_MOTOR_REVERSED.";
            return false;
        }
        if (kS < 0) kS = 0; // a negative intercept just means the sweep did not reach low enough
        return true;
    }

    /** Holds an open-loop power, lets it settle, then returns the average RPM. NaN if it gave up. */
    private double measureSteadyRpm(double power) {
        ElapsedTime clock = new ElapsedTime();
        double sum = 0;
        int samples = 0;
        double settle = RobotConfig.TUNE_SHOOTER_FF_SETTLE_S;

        while (opModeIsActive() && !gamepad1.b
                && clock.seconds() < settle + RobotConfig.TUNE_SHOOTER_FF_SAMPLE_S) {
            shooter.setOpenLoopPower(power);
            shooter.updateMeasurement();

            if (shooter.getRpm() > RobotConfig.TUNE_SHOOTER_ABORT_RPM) {
                failure = String.format("RPM %.0f passed SHOOTER_ABORT_RPM (%.0f) at power %.2f.",
                        shooter.getRpm(), RobotConfig.TUNE_SHOOTER_ABORT_RPM, power);
                shooter.stop();
                return Double.NaN;
            }
            if (clock.seconds() >= settle) {
                sum += shooter.getRpm();
                samples++;
            }

            telemetry.addLine("Phase 1/2: feedforward sweep");
            telemetry.addData("power", "%.2f", power);
            telemetry.addData("rpm", "%.0f", shooter.getRpm());
            telemetry.addData("sampling", "%d", samples);
            telemetry.update();
        }

        shooter.setOpenLoopPower(0);
        if (gamepad1.b) failure = "B was pressed.";
        else if (samples == 0) failure = String.format("No RPM samples at power %.2f.", power);
        return (samples == 0) ? Double.NaN : sum / samples;
    }

    // ---------------------------------------------------------------------
    // Phase 2: relay test (Ku / Tu)
    // ---------------------------------------------------------------------

    /** Runs the library relay test around the shooting speed. Null (with failure set) if it cannot. */
    private boolean relayTest() {
        // Bias the relay to the feedforward power that holds the target, so the flywheel oscillates
        // around the shooting speed instead of around zero power. The relay still swings by
        // +/- SHOOTER_RELAY_AMPLITUDE, which is the amplitude the library's Ku formula expects.
        double bias = kS + kV * RobotConfig.TUNE_SHOOTER_TARGET_RPM;
        double amplitude = RobotConfig.TUNE_SHOOTER_RELAY_AMPLITUDE;

        // The relay has to swing symmetrically, or Ku is wrong: check both ends before spinning up.
        if (bias + amplitude > 1.0) {
            failure = String.format("The flywheel needs %.2f power for %.0f RPM, so a %.2f relay swing does not fit "
                            + "under 1.0. Lower SHOOTER_TARGET_RPM or SHOOTER_RELAY_AMPLITUDE.",
                    bias, RobotConfig.TUNE_SHOOTER_TARGET_RPM, amplitude);
            return false;
        }
        if (bias - amplitude < 0.0) {
            failure = String.format("A %.2f relay swing around %.2f power would clip at 0. Raise SHOOTER_TARGET_RPM "
                            + "or lower SHOOTER_RELAY_AMPLITUDE.", amplitude, bias);
            return false;
        }

        RelayAutoTuner tuner = new RelayAutoTuner(
                RobotConfig.TUNE_SHOOTER_TARGET_RPM,
                amplitude,
                RobotConfig.TUNE_SHOOTER_HYSTERESIS_RPM,
                RobotConfig.TUNE_SHOOTER_CYCLES_TO_COLLECT,
                RobotConfig.TUNE_SHOOTER_CYCLES_TO_IGNORE);

        ElapsedTime clock = new ElapsedTime();
        int totalCycles = RobotConfig.TUNE_SHOOTER_CYCLES_TO_COLLECT + RobotConfig.TUNE_SHOOTER_CYCLES_TO_IGNORE;

        while (opModeIsActive() && !tuner.isFinished()) {
            shooter.updateMeasurement();
            double rpm = shooter.getRpm();

            if (gamepad1.b) {
                failure = "B was pressed.";
                break;
            }
            if (rpm > RobotConfig.TUNE_SHOOTER_ABORT_RPM) {
                failure = "RPM passed SHOOTER_ABORT_RPM.";
                break;
            }

            double relay = tuner.update(rpm, clock.seconds());
            shooter.setOpenLoopPowerSigned(bias + relay);

            telemetry.addLine("Phase 2/2: relay test (Ziegler-Nichols)");
            telemetry.addData("rpm / target", "%.0f / %.0f", rpm, RobotConfig.TUNE_SHOOTER_TARGET_RPM);
            telemetry.addData("cycles", "%d / %d", tuner.cyclesCollected(), totalCycles);
            telemetry.addData("elapsed", "%.1f s", clock.seconds());
            telemetry.update();

            if (clock.seconds() > RobotConfig.TUNE_SHOOTER_TIMEOUT_S) {
                failure = "TIMED OUT: no stable oscillation. Raise SHOOTER_RELAY_AMPLITUDE or lower SHOOTER_TARGET_RPM.";
                break;
            }
        }

        shooter.stop();
        if (failure != null) return false;

        relayResult = tuner.computeResult();
        if (relayResult == null) {
            failure = "The relay test saw no usable oscillation amplitude. Check the encoder.";
            return false;
        }
        candidates = ZieglerNicholsCalculator.computeCandidates(relayResult, kV, RobotConfig.TUNE_SHOOTER_TUNE_INTEGRAL);
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
        int selected = Math.max(0, Math.min(candidates.size() - 1, RobotConfig.TUNE_SHOOTER_LIVE_CANDIDATE));
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
                    live.setOutputBounds(0.0, 1.0);
                    liveIndex = selected;
                }
                shooter.updateMeasurement();
                // kS is not part of the library's controller, so add it here: the flywheel sees the
                // same feedforward + PID the Shooter controller will run once these gains are applied.
                double power = kS + live.calculate(shooter.getRpm(), RobotConfig.TUNE_SHOOTER_TARGET_RPM);
                shooter.setOpenLoopPower(Math.max(0.0, Math.min(1.0, power)));
            } else {
                live = null;
                liveIndex = -1;
                shooter.setOpenLoopPower(0);
            }

            boolean b = gamepad1.b;
            if (b && !previousB) {
                RobotConfig.SHOOTER_kP = gains.kP;
                RobotConfig.SHOOTER_kI = gains.kI;
                RobotConfig.SHOOTER_kD = gains.kD;
                RobotConfig.SHOOTER_kV = gains.kF;
                RobotConfig.SHOOTER_kS = kS;
                applied = gains.label;
            }
            previousB = b;

            telemetry.addLine("FTC-AutoTune: flywheel velocity");
            telemetry.addData("Ku / Tu", "%.5f / %.4f s", relayResult.Ku, relayResult.Tu);
            telemetry.addData("kS / kV", "%.5f / %.6f", kS, kV);
            for (int i = 0; i < candidates.size(); i++) {
                telemetry.addLine((i == selected ? "> " : "  ") + candidates.get(i));
            }
            telemetry.addData("rpm / target", "%.0f / %.0f", shooter.getRpm(), RobotConfig.TUNE_SHOOTER_TARGET_RPM);
            telemetry.addData("applied to Shooter", applied);
            telemetry.addLine("D-pad up/down: pick   A: hold = live test   B: apply");
            telemetry.update();
        }
        shooter.stop();
    }

    private void showFailure() {
        shooter.stop();
        while (opModeIsActive()) {
            telemetry.addLine("TUNING STOPPED");
            telemetry.addLine(failure == null ? "The OpMode was stopped." : failure);
            telemetry.update();
        }
    }
}
