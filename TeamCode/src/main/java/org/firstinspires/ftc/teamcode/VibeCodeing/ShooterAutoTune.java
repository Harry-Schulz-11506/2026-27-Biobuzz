package org.firstinspires.ftc.teamcode.VibeCodeing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autotune for the flywheel velocity loop, in two phases:
 *  1) Feedforward: runs two open-loop powers, measures RPM, and solves kS and kV.
 *  2) Relay feedback around TUNE_RPM to find Ku/Tu, giving kP and kI (kD = 0).
 *
 * Make sure nothing is in the shooter path. Press B to abort.
 * When finished: A = apply gains (live in Dashboard's "Shooter" config). Copy them into Shooter.java to keep them.
 */
@Config
@TeleOp(name = "AutoTune: Shooter", group = "Tuning")
public class ShooterAutoTune extends LinearOpMode {

    public static double TUNE_RPM = 2200;
    public static double FF_POWER_LOW = 0.35;
    public static double FF_POWER_HIGH = 0.80;
    public static double FF_SETTLE_SEC = 5.0; // the heavier flywheel needs longer to settle
    public static double FF_SAMPLE_SEC = 1.0;
    public static double RELAY_DELTA_POWER = 0.08;
    public static double HYSTERESIS_RPM = 40;
    public static int WARMUP_CYCLES = 2;
    public static int MEASURE_CYCLES = 5;
    public static double TIMEOUT_SEC = 60;
    public static double ABORT_RPM = 5500; // the motor free speed is 6000 RPM and the ratio is 1:1

    private Shooter shooter;
    private String abortReason = "";      // why measureRpmAtPower() gave up (shown in the failure message)

    /** Hold an open-loop power, wait to settle, then return the average RPM. Returns NaN if aborted. */
    private double measureRpmAtPower(double power) {
        ElapsedTime t = new ElapsedTime();
        double sum = 0;
        int n = 0;
        while (opModeIsActive() && !gamepad1.b && t.seconds() < FF_SETTLE_SEC + FF_SAMPLE_SEC) {
            shooter.setOpenLoopPower(power);
            shooter.updateMeasurement();
            if (shooter.getRpm() > ABORT_RPM) {
                abortReason = String.format("RPM %.0f went above ABORT_RPM (%.0f) at power %.2f", shooter.getRpm(), ABORT_RPM, power);
                return Double.NaN;
            }
            if (t.seconds() > FF_SETTLE_SEC) {
                sum += shooter.getRpm();
                n++;
            }
            telemetry.addData("phase", "1: feedforward");
            telemetry.addData("power", power);
            telemetry.addData("rpm", shooter.getRpm());
            telemetry.update();
        }
        if (gamepad1.b) abortReason = "B was pressed";
        else if (n == 0) abortReason = String.format("no RPM samples at power %.2f (RPM stayed at %.0f)", power, shooter.getRpm());
        return (n == 0) ? Double.NaN : sum / n;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Shooter();
        shooter.init(hardwareMap);

        telemetry.addLine("Clear the shooter path, then press START. B = abort.");
        telemetry.update();
        waitForStart();

        String failure = null;
        double kS = 0, kV = 0;
        RelayAutoTuner tuner = null;

        // ---------- Phase 1: feedforward ----------
        shooter.updateMeasurement();
        double rpmLow = measureRpmAtPower(FF_POWER_LOW);
        double vLow = shooter.getVoltage();
        double rpmHigh = Double.isNaN(rpmLow) ? Double.NaN : measureRpmAtPower(FF_POWER_HIGH);
        double vHigh = shooter.getVoltage();

        if (Double.isNaN(rpmLow) || Double.isNaN(rpmHigh)) {
            failure = "Phase 1 stopped: " + abortReason + ". If RPM went too high, raise ABORT_RPM or lower FF_POWER_HIGH; "
                    + "if RPM read 0, check Shooter.REVERSED, the encoder cable and the motor name 'shooter'.";
        } else if (rpmHigh - rpmLow < 100) {
            failure = String.format("RPM barely changed between the two powers (%.0f at %.2f, %.0f at %.2f). "
                    + "Check the encoder cable and the motor.", rpmLow, FF_POWER_LOW, rpmHigh, FF_POWER_HIGH);
        } else {
            // Normalise power to nominal voltage: P * V / Vnom = kS + kV * rpm
            double pLow = FF_POWER_LOW * vLow / Shooter.NOMINAL_VOLTAGE;
            double pHigh = FF_POWER_HIGH * vHigh / Shooter.NOMINAL_VOLTAGE;
            kV = (pHigh - pLow) / (rpmHigh - rpmLow);
            kS = pLow - kV * rpmLow;
            if (kV <= 0) failure = "Computed kV <= 0; measurement was bad.";
            if (kS < 0) kS = 0;
        }

        // ---------- Phase 2: relay feedback ----------
        if (failure == null && opModeIsActive()) {
            double bias = (kS + kV * TUNE_RPM) * (Shooter.NOMINAL_VOLTAGE / shooter.getVoltage());
            tuner = new RelayAutoTuner(TUNE_RPM, RELAY_DELTA_POWER, HYSTERESIS_RPM,
                    WARMUP_CYCLES, MEASURE_CYCLES, TIMEOUT_SEC);
            ElapsedTime clock = new ElapsedTime();

            while (opModeIsActive() && tuner.getStatus() == RelayAutoTuner.Status.RUNNING) {
                shooter.updateMeasurement();
                double rpm = shooter.getRpm();
                if (gamepad1.b) { tuner.fail("Aborted by driver."); break; }
                if (rpm > ABORT_RPM) { tuner.fail("RPM exceeded ABORT_RPM."); break; }

                double dir = tuner.update(rpm, clock.seconds());
                shooter.setOpenLoopPower(bias + dir * RELAY_DELTA_POWER);

                telemetry.addData("phase", "2: relay");
                telemetry.addData("target rpm", TUNE_RPM);
                telemetry.addData("rpm", rpm);
                telemetry.addData("cycles", tuner.getMeasuredCycles() + " / " + MEASURE_CYCLES);
                telemetry.update();
            }
            if (tuner.getStatus() == RelayAutoTuner.Status.FAILED) failure = tuner.getFailReason();
        }

        shooter.stop();

        RelayAutoTuner.Gains pi = (failure == null) ? tuner.tyreusLuybenPI() : null;

        while (opModeIsActive()) {
            if (failure != null) {
                telemetry.addLine("FAILED: " + failure);
            } else {
                if (gamepad1.a) {
                    Shooter.kS = kS;
                    Shooter.kV = kV;
                    Shooter.kP = pi.kP;
                    Shooter.kI = pi.kI;
                    Shooter.kD = 0;
                }
                telemetry.addLine("DONE.  A = apply gains");
                telemetry.addData("kS", "%.5f", kS);
                telemetry.addData("kV", "%.6f", kV);
                telemetry.addData("Ku / Tu(s)", "%.6f / %.4f", tuner.getKu(), tuner.getTu());
                telemetry.addData("kP / kI", "%.6f / %.6f", pi.kP, pi.kI);
                telemetry.addData("live kP / kI / kV / kS", "%.6f / %.6f / %.6f / %.5f",
                        Shooter.kP, Shooter.kI, Shooter.kV, Shooter.kS);
            }
            telemetry.update();
        }
    }
}