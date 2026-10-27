package org.firstinspires.ftc.teamcode.VibeCodeing;

/**
 * Relay-feedback (Astrom-Hagglund) auto-tuner.
 *
 * Usage: call update(measurement, timeSeconds) every loop. It returns +1 or -1, which the
 * caller scales by the relay amplitude and adds to any bias/feedforward. When getStatus()
 * becomes DONE, getKu() / getTu() hold the ultimate gain and period, and the helper methods
 * convert them into PID gains (parallel form: out = kP*e + kI*integral(e) + kD*de/dt).
 */
public class RelayAutoTuner {

    public enum Status { RUNNING, DONE, FAILED }

    public static class Gains {
        public final double kP, kI, kD;
        public Gains(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    private final double setpoint;
    private final double outputAmplitude; // relay output swing, same units as the caller's output
    private final double hysteresis;      // measurement units
    private final int warmupCycles;
    private final int measureCycles;
    private final double timeoutSec;

    private Status status = Status.RUNNING;
    private String failReason = "";

    private boolean started = false;
    private double startTime = 0;
    private boolean outputHigh = true;
    private double lastRiseTime = Double.NaN;
    private double windowMax = Double.NEGATIVE_INFINITY;
    private double windowMin = Double.POSITIVE_INFINITY;
    private int cyclesSeen = 0;
    private int measured = 0;
    private double sumPeriod = 0;
    private double sumAmplitude = 0;

    private double ku = Double.NaN;
    private double tu = Double.NaN;

    public RelayAutoTuner(double setpoint, double outputAmplitude, double hysteresis,
                          int warmupCycles, int measureCycles, double timeoutSec) {
        this.setpoint = setpoint;
        this.outputAmplitude = outputAmplitude;
        this.hysteresis = hysteresis;
        this.warmupCycles = warmupCycles;
        this.measureCycles = measureCycles;
        this.timeoutSec = timeoutSec;
    }

    /** @return +1 (push measurement up) or -1 (push it down); 0 once finished or failed. */
    public double update(double measurement, double nowSec) {
        if (status != Status.RUNNING) return 0;

        if (!started) {
            started = true;
            startTime = nowSec;
            outputHigh = measurement < setpoint;
        }

        if (nowSec - startTime > timeoutSec) {
            fail("Timed out: no stable oscillation. Try a larger relay amplitude or smaller hysteresis.");
            return 0;
        }

        windowMax = Math.max(windowMax, measurement);
        windowMin = Math.min(windowMin, measurement);

        if (outputHigh && measurement > setpoint + hysteresis) {
            outputHigh = false;
        } else if (!outputHigh && measurement < setpoint - hysteresis) {
            outputHigh = true;
            // A full cycle just completed (low -> high switch to low -> high switch).
            if (!Double.isNaN(lastRiseTime)) {
                cyclesSeen++;
                if (cyclesSeen > warmupCycles) {
                    sumPeriod += nowSec - lastRiseTime;
                    sumAmplitude += (windowMax - windowMin) / 2.0;
                    measured++;
                }
            }
            lastRiseTime = nowSec;
            windowMax = measurement;
            windowMin = measurement;

            if (measured >= measureCycles) {
                finish();
                return 0;
            }
        }

        return outputHigh ? 1.0 : -1.0;
    }

    private void finish() {
        double avgPeriod = sumPeriod / measured;
        double avgAmplitude = sumAmplitude / measured;
        if (avgAmplitude <= 1e-9 || avgPeriod <= 1e-6) {
            fail("Measured amplitude or period was ~0.");
            return;
        }
        ku = 4.0 * outputAmplitude / (Math.PI * avgAmplitude);
        tu = avgPeriod;
        status = Status.DONE;
    }

    public void fail(String reason) {
        status = Status.FAILED;
        failReason = reason;
    }

    public Status getStatus() { return status; }
    public String getFailReason() { return failReason; }
    public int getMeasuredCycles() { return measured; }
    public double getKu() { return ku; }
    public double getTu() { return tu; }

    // ---- Tuning rules (valid once DONE) ----

    /** Conservative PID (less overshoot). Recommended starting point. */
    public Gains tyreusLuybenPID() {
        double kp = ku / 2.2;
        double ti = 2.2 * tu;
        double td = tu / 6.3;
        return new Gains(kp, kp / ti, kp * td);
    }

    /** Conservative PI (no derivative). Good for velocity loops. */
    public Gains tyreusLuybenPI() {
        double kp = ku / 3.2;
        double ti = 2.2 * tu;
        return new Gains(kp, kp / ti, 0);
    }

    /** Classic Ziegler-Nichols PID (aggressive, will overshoot). */
    public Gains zieglerNicholsPID() {
        double kp = 0.6 * ku;
        double ti = tu / 2.0;
        double td = tu / 8.0;
        return new Gains(kp, kp / ti, kp * td);
    }
}
