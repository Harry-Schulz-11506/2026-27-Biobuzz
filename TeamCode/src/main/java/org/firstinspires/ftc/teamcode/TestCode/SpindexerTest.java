package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Spindexer TeleOp
 *  - 2x REV Color Sensor V3 plugged straight into the hub
 *  - Optii Encoder V2 (4096 counts/rev) on the spindexer shaft, plugged into a hub encoder port.
 *    The code drives the motor with its own position controller using that encoder,
 *    so gear ratios no longer matter.
 *
 * Positions (from above, front at the bottom). Balls travel around the ring 1 -> 2 -> 4 -> 3 -> 1
 *        POS 3 (YELLOW shooter)   POS 4 (ORANGE shooter)
 *        POS 1 (ColorA)           POS 2 (ColorB)
 *
 * INTAKE : ColorA watches position 1, ColorB watches position 2. The spindexer does NOT move until
 *          BOTH have a ball, then it turns 180 deg so the two empty slots come around
 *          (the two loaded balls end up at positions 4 and 3).
 * Y      : ejects every loaded ball. Orange -> position 4, Yellow -> position 3.
 *          If an orange and a yellow sit in neighbouring slots it fires BOTH at the same time;
 *          leftover balls go one at a time. Holds at the exit(s) for 3 s per shot.
 *
 * Controls: Y = eject | D-pad left/right = nudge, then B = re-zero (this is aligned)
 *           | BACK = forget all balls (mark every slot empty)
 */
@TeleOp(name = "Spindexer")
public class SpindexerTest extends LinearOpMode {

    // ================= CONFIG - CHANGE THESE =================
    // Hardware config names
    static final String MOTOR_NAME    = "spindexer";
    // Optii encoder: if it is plugged into the SAME hub port as the spindexer motor (in place of the
    // motor's own encoder cable) leave this equal to MOTOR_NAME. If it is on a different port,
    // add that port as a motor in the config and put its name here (e.g. "spindexerEncoder").
    static final String ENCODER_NAME  = "spindexer";
    static final String SENSOR_A_NAME = "colorA";   // watches slot 1
    static final String SENSOR_B_NAME = "colorB";   // watches slot 2

    // The 4 positions around the ring, in order: pos 1 (0 deg), pos 2 (90), pos 4 (180), pos 3 (270).
    // "Positive" = a ball moves from position 1 toward position 2.
    static final double SENSOR_B_SLOT_DEG = 90;    // position 2 (ColorB)
    // Fine-tune how exactly the balls line up with BOTH shooters (degrees, try +/- 1 to 3).
    // Positive = the spindexer stops a little further in the "positive" direction.
    static final double EXIT_TRIM_DEG     = 0;
    static final double EXIT_RIGHT_DEG    = 180 + EXIT_TRIM_DEG;   // position 4 - ORANGE shooter
    static final double EXIT_LEFT_DEG     = 270 + EXIT_TRIM_DEG;   // position 3 - YELLOW shooter
    // How many slots ahead of the orange ball the yellow ball must be so both sit at their shooters
    // at once (270 - 180 = 90 deg = 1 slot). Used to fire orange + yellow together.
    static final int PAIR_OFFSET = (((int) Math.round((EXIT_LEFT_DEG - EXIT_RIGHT_DEG) / 90.0)) % 4 + 4) % 4;

    // Colour detection (tune using the telemetry: hue and dist)
    static final float  SENSOR_GAIN     = 2.0f;
    static final double BALL_DIST_CM    = 11.0;
    static final float  MIN_SATURATION  = 0.35f;
    static final float  ORANGE_HUE_MIN  = 50,  ORANGE_HUE_MAX = 70;
    static final float  YELLOW_HUE_MIN  = 100, YELLOW_HUE_MAX = 140;

    // Encoder (on the spindexer shaft, so 1 encoder rev = 1 spindexer rev)
    static final double ENCODER_CPR = 4096;

    // TWO independent switches (each +1 or -1). Use "Spindexer Encoder Test" to set them:
    //  MOTOR_SIGN        : flip if pushing "forward" moves a ball position 1 -> position 3 instead of 1 -> 2.
    //  ENCODER_DIRECTION : flip if the "Position" reading goes DOWN while the ball moves position 1 -> 2.
    // (Wrong ENCODER_DIRECTION = the code stops the motor and shows an "encoder direction wrong" message.)
    static final int MOTOR_SIGN = 1;
    static final int ENCODER_DIRECTION = 1;

    // Motor
    static final double MOTOR_POWER = 1;        // max power
    static final double KP = 0.004;               // power per encoder tick of error
    static final double KD = 0.00003;             // damping (power per tick/s)
    static final double KS = 0.06;                // minimum power to get moving
    static final int    TOLERANCE_TICKS = 15;     // ~1.3 degrees (smaller = more exact, too small = hunting)
    static final double SETTLE_SEC = 0.20;        // must stay in tolerance this long to count as "arrived"
    static final double VELOCITY_TOL = 80;        // ticks/s - must also be nearly stopped
    static final double MOVE_TIMEOUT_SEC = 4.0;   // gives up on a move so it can never get stuck

    static final double EXIT_HOLD_SEC = 3.0;      // wait at each exit
    // Colour reading accuracy: a ball only counts after a voting window of readings that agree
    static final double DETECT_SEC    = 0.25;     // length of the voting window
    static final int    MIN_SAMPLES   = 5;        // at least this many good readings in the window
    static final double MIN_AGREE     = 0.85;     // this fraction of them must say the same colour
    static final int    MAX_MISSES    = 3;        // this many "no reading" loops in a row restarts the window
    static final int    NUDGE_TICKS   = 30;       // ~2.6 degrees per press
    // =========================================================

    static final double TICKS_PER_DEG = ENCODER_CPR / 360.0;

    static final int EMPTY = 0, ORANGE = 1, YELLOW = 2;

    enum State { INTAKE, INDEXING, EXIT_MOVE, EXIT_HOLD, RETURN }

    private DcMotorEx spindexer;
    private DcMotorEx encoder;
    private NormalizedColorSensor sensorA, sensorB;

    private final int[] slots = new int[4];     // what's in each physical slot
    private long targetTicks = 0;
    private State state = State.INTAKE;

    // position controller state
    private boolean inTolerance = false;
    private final ElapsedTime settleTimer = new ElapsedTime();
    private double moveStartAbsError = 0;
    private boolean faulted = false;
    private double lastError = 0;
    private double lastPower = 0;

    private final int[][] votes = new int[2][3];   // per sensor: readings that said ORANGE(1) / YELLOW(2)
    private final int[] samples = new int[2];
    private final int[] misses = new int[2];
    private final ElapsedTime[] detectTimer = { new ElapsedTime(), new ElapsedTime() };
    private final ElapsedTime holdTimer = new ElapsedTime();
    private final ElapsedTime moveTimer = new ElapsedTime();
    private final List<int[]> shots = new ArrayList<>();   // each shot = {orangeSlot, yellowSlot}, -1 = none
    private int[] currentShot = null;
    private String message = "";

    // last readings for telemetry
    private final float[] hueShown = new float[2];
    private final double[] distShown = new double[2];
    private final int[] colorShown = new int[2];
    private final String[] readingShown = { "-", "-" };

    @Override
    public void runOpMode() {
        sensorA = hardwareMap.get(NormalizedColorSensor.class, SENSOR_A_NAME);
        sensorB = hardwareMap.get(NormalizedColorSensor.class, SENSOR_B_NAME);
        sensorA.setGain(SENSOR_GAIN);
        sensorB.setGain(SENSOR_GAIN);

        spindexer = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoder = ENCODER_NAME.equals(MOTOR_NAME) ? spindexer : hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        zeroEncoder();

        telemetry.addLine("Put 2 EMPTY slots at slot 1 / slot 2, then press START.");
        telemetry.addData("Encoder ticks / rev", "%.0f", ENCODER_CPR);
        telemetry.update();
        waitForStart();

        boolean prevY = false, prevLeft = false, prevRight = false, prevB = false, prevBack = false;

        while (opModeIsActive()) {
            updateController();   // runs every loop: drives to target and holds position

            boolean yPressed     = gamepad1.y && !prevY;
            boolean leftPressed  = gamepad1.dpad_left && !prevLeft;
            boolean rightPressed = gamepad1.dpad_right && !prevRight;
            boolean bPressed     = gamepad1.b && !prevB;
            boolean backPressed  = gamepad1.back && !prevBack;
            prevY = gamepad1.y;
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;
            prevB = gamepad1.b;
            prevBack = gamepad1.back;

            switch (state) {
                case INTAKE:
                    if (leftPressed)  setTarget(targetTicks - NUDGE_TICKS);
                    if (rightPressed) setTarget(targetTicks + NUDGE_TICKS);
                    if (bPressed) zeroEncoder();
                    if (backPressed) clearMemory();

                    if (yPressed) startSort();
                    else runIntake();
                    break;

                case INDEXING:
                case RETURN:
                    readSensorsForTelemetry();
                    if (moveDone()) state = State.INTAKE;
                    break;

                case EXIT_MOVE:
                    readSensorsForTelemetry();
                    if (moveDone()) {
                        holdTimer.reset();
                        state = State.EXIT_HOLD;
                    }
                    break;

                case EXIT_HOLD:
                    readSensorsForTelemetry();
                    if (holdTimer.seconds() >= EXIT_HOLD_SEC) {
                        if (currentShot[0] >= 0) slots[currentShot[0]] = EMPTY;   // balls are gone
                        if (currentShot[1] >= 0) slots[currentShot[1]] = EMPTY;
                        nextExit();
                    }
                    break;
            }

            showTelemetry();
        }
        spindexer.setPower(0);
    }

    // ---------------- position controller (uses the Optii encoder) ----------------

    private double position() {
        return ENCODER_DIRECTION * encoder.getCurrentPosition();
    }

    private void updateController() {
        double pos = position();
        double error = targetTicks - pos;
        lastError = error;

        if (faulted) {
            spindexer.setPower(0);
            lastPower = 0;
            return;
        }

        // If the error is GROWING right after a move starts, the encoder sign is wrong: stop.
        if (!inTolerance && moveTimer.seconds() > 0.6 && Math.abs(error) > moveStartAbsError + 150) {
            faulted = true;
            spindexer.setPower(0);
            lastPower = 0;
            return;
        }

        double power;
        if (Math.abs(error) <= TOLERANCE_TICKS) {
            if (!inTolerance) { inTolerance = true; settleTimer.reset(); }
            power = 0;
        } else {
            inTolerance = false;
            double vel = ENCODER_DIRECTION * encoder.getVelocity();   // ticks per second
            power = KP * error - KD * vel + Math.signum(error) * KS;
            power = Math.max(-MOTOR_POWER, Math.min(MOTOR_POWER, power));
        }
        spindexer.setPower(MOTOR_SIGN * power);
        lastPower = power;
    }

    private boolean atTarget() {
        return inTolerance && settleTimer.seconds() >= SETTLE_SEC
                && Math.abs(encoder.getVelocity()) < VELOCITY_TOL;
    }

    /** True when the spindexer arrived, or the move took too long / faulted (so we never get stuck). */
    private boolean moveDone() {
        if (atTarget()) return true;
        if (faulted) return true;
        if (moveTimer.seconds() > MOVE_TIMEOUT_SEC) {
            message = "Move timed out - check motor/jam";
            return true;
        }
        return false;
    }

    private void setTarget(long ticks) {
        targetTicks = ticks;
        inTolerance = false;
        moveStartAbsError = Math.abs(targetTicks - position());
        moveTimer.reset();
    }

    private void indexDegrees(double deg) {
        setTarget(targetTicks + Math.round(deg * TICKS_PER_DEG));
        state = State.INDEXING;
    }

    private void zeroEncoder() {
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // we control power ourselves
        if (encoder != spindexer) {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        targetTicks = 0;
        inTolerance = false;
        moveStartAbsError = 0;
        moveTimer.reset();
    }

    // ---------------- intake ----------------

    private void runIntake() {
        int slotA = slotAtAngle(0);
        int slotB = slotAtAngle(SENSOR_B_SLOT_DEG);
        int[] targetSlot = { slotA, slotB };
        int[] seen = { classify(sensorA, 0), classify(sensorB, 1) };
        boolean settled = atTarget();          // only read balls while the spindexer is stopped

        boolean filledNow = false;
        for (int k = 0; k < 2; k++) {
            int s = targetSlot[k];
            if (slots[s] != EMPTY || !settled) { resetVotes(k); continue; }

            if (seen[k] == 0) {                // no ball / unsure this loop
                if (++misses[k] > MAX_MISSES) resetVotes(k);
                continue;
            }
            misses[k] = 0;
            if (samples[k] == 0) detectTimer[k].reset();
            votes[k][seen[k]]++;
            samples[k]++;

            if (detectTimer[k].seconds() >= DETECT_SEC && samples[k] >= MIN_SAMPLES) {
                int winner = votes[k][ORANGE] >= votes[k][YELLOW] ? ORANGE : YELLOW;
                double agree = votes[k][winner] / (double) samples[k];
                if (agree >= MIN_AGREE) {      // clear majority -> accept
                    slots[s] = winner;
                    filledNow = true;
                }
                resetVotes(k);                 // otherwise: not sure, start over
            }
        }

        boolean bothFull = slots[slotA] != EMPTY && slots[slotB] != EMPTY;
        int have = (slots[slotA] != EMPTY ? 1 : 0) + (slots[slotB] != EMPTY ? 1 : 0);

        if (bothFull && filledNow) {
            message = "Got 2 balls - turning 180";
            indexDegrees(180);   // bring the 2 empty slots around
        } else if (bothFull) {
            message = "Intake slots full - press Y to eject";
        } else {
            message = "Waiting for 2 balls (have " + have + ")";
        }
    }

    private void resetVotes(int k) {
        votes[k][ORANGE] = 0;
        votes[k][YELLOW] = 0;
        samples[k] = 0;
        misses[k] = 0;
    }

    private void clearMemory() {
        Arrays.fill(slots, EMPTY);
        resetVotes(0);
        resetVotes(1);
    }

    // ---------------- colour sensing ----------------

    /** Reads one sensor. Returns ORANGE, YELLOW or 0 (nothing / unsure). Hue in the gap = unsure. */
    private int classify(NormalizedColorSensor sensor, int idx) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(rgba.toColor(), hsv);
        double dist = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);

        hueShown[idx] = hsv[0];
        distShown[idx] = dist;

        int result = 0;
        String label;
        if (Double.isNaN(dist) || dist > BALL_DIST_CM) {
            label = "no ball";
        } else if (hsv[1] < MIN_SATURATION) {
            label = "UNSURE (too grey)";
        } else if (hsv[0] >= ORANGE_HUE_MIN && hsv[0] <= ORANGE_HUE_MAX) {
            result = ORANGE;
            label = "ORANGE";
        } else if (hsv[0] >= YELLOW_HUE_MIN && hsv[0] <= YELLOW_HUE_MAX) {
            result = YELLOW;
            label = "YELLOW";
        } else {
            label = "UNSURE (hue in gap)";
        }
        colorShown[idx] = result;
        readingShown[idx] = label;
        return result;
    }

    private void readSensorsForTelemetry() {
        classify(sensorA, 0);
        classify(sensorB, 1);
    }

    // ---------------- sorting / ejecting ----------------

    private void startSort() {
        shots.clear();
        boolean[] used = new boolean[4];

        // 1) Pair every orange with the yellow sitting one slot ahead of it (fires both shooters at once)
        for (int o = 0; o < 4; o++) {
            int y = (o + PAIR_OFFSET) % 4;
            if (slots[o] == ORANGE && slots[y] == YELLOW && !used[o] && !used[y]) {
                shots.add(new int[]{ o, y });
                used[o] = used[y] = true;
            }
        }
        // 2) Anything left goes out on its own (orange first, then yellow)
        for (int i = 0; i < 4; i++) if (slots[i] == ORANGE && !used[i]) shots.add(new int[]{ i, -1 });
        for (int i = 0; i < 4; i++) if (slots[i] == YELLOW && !used[i]) shots.add(new int[]{ -1, i });

        if (shots.isEmpty()) { message = "Nothing loaded to eject"; return; }
        message = "Ejecting " + shots.size() + " shot(s)";
        nextExit();
    }

    private void nextExit() {
        if (shots.isEmpty()) {
            resetVotes(0);
            resetVotes(1);
            double thetaDeg = targetTicks / TICKS_PER_DEG;
            double alignedDeg = Math.round(thetaDeg / 90.0) * 90.0;   // nearest slot-at-front position
            setTarget(Math.round(alignedDeg * TICKS_PER_DEG));
            state = State.RETURN;
            message = "Returning to intake";
            return;
        }
        currentShot = pickNextShot();
        boolean both = currentShot[0] >= 0 && currentShot[1] >= 0;
        if (both) message = "Balls " + slotLetter(currentShot[0]) + " + " + slotLetter(currentShot[1]) + ": ORANGE -> pos 4 + YELLOW -> pos 3 together";
        else if (currentShot[0] >= 0) message = "Ball " + slotLetter(currentShot[0]) + " (orange) -> pos 4";
        else message = "Ball " + slotLetter(currentShot[1]) + " (yellow) -> pos 3";

        setTarget(targetTicks + Math.round(shotDelta(currentShot) * TICKS_PER_DEG));
        state = State.EXIT_MOVE;
    }

    /** Degrees to rotate (shortest way) so this shot's ball(s) sit at their shooter(s). */
    private double shotDelta(int[] shot) {
        double thetaDeg = targetTicks / TICKS_PER_DEG;
        if (shot[0] >= 0) return wrap180((EXIT_RIGHT_DEG - shot[0] * 90.0) - thetaDeg);
        return wrap180((EXIT_LEFT_DEG - shot[1] * 90.0) - thetaDeg);
    }

    /** Pair shots first (closest one first), then single balls in the order they were queued. */
    private int[] pickNextShot() {
        int bestIdx = -1;
        double bestMove = 1e9;
        for (int i = 0; i < shots.size(); i++) {
            int[] s = shots.get(i);
            if (s[0] >= 0 && s[1] >= 0) {
                double move = Math.abs(shotDelta(s));
                if (move < bestMove) { bestMove = move; bestIdx = i; }
            }
        }
        if (bestIdx < 0) bestIdx = 0;
        return shots.remove(bestIdx);
    }

    // ---------------- slot tracking ----------------

    /** Angle of physical slot i measured from slot 1's position, 0-360. Slot i starts at i*90 deg. */
    private double ringAngle(int slot) {
        double a = (slot * 90.0 + targetTicks / TICKS_PER_DEG) % 360.0;
        return a < 0 ? a + 360.0 : a;
    }

    /** Which physical slot is currently closest to the given angle. */
    private int slotAtAngle(double deg) {
        int best = 0;
        double bestDiff = 1e9;
        for (int i = 0; i < 4; i++) {
            double diff = Math.abs(wrap180(ringAngle(i) - deg));
            if (diff < bestDiff) { bestDiff = diff; best = i; }
        }
        return best;
    }

    private static double wrap180(double x) {
        x = x % 360.0;
        if (x > 180) x -= 360;
        if (x <= -180) x += 360;
        return x;
    }

    // ---------------- telemetry ----------------

    private String colorName(int c) {
        return c == ORANGE ? "ORANGE" : c == YELLOW ? "YELLOW" : "empty";
    }

    private String slotLetter(int i) {
        return String.valueOf((char) ('A' + i));
    }

    private String positionName(double angle) {
        if (Math.abs(wrap180(angle)) < 10) return "POS 1 (ColorA)";
        if (Math.abs(wrap180(angle - SENSOR_B_SLOT_DEG)) < 10) return "POS 2 (ColorB)";
        if (Math.abs(wrap180(angle - EXIT_RIGHT_DEG)) < 10) return "POS 4 (ORANGE shooter)";
        if (Math.abs(wrap180(angle - EXIT_LEFT_DEG)) < 10) return "POS 3 (YELLOW shooter)";
        return String.format("moving (%.0f deg)", angle);
    }

    private void showTelemetry() {
        if (faulted) {
            telemetry.addLine("!!! ENCODER DIRECTION WRONG - motor stopped !!!");
            telemetry.addLine("Flip ENCODER_DIRECTION (and check the encoder is plugged in), then restart. Use Spindexer Encoder Test.");
        }
        telemetry.addData("State", state + (atTarget() ? "" : " (moving)"));
        telemetry.addData("Status", message);
        telemetry.addLine();
        for (int i = 0; i < 4; i++) {
            telemetry.addData("Ball slot " + slotLetter(i), "%s  @ %s", colorName(slots[i]), positionName(ringAngle(i)));
        }
        telemetry.addLine();
        telemetry.addData("ColorA (pos 1)", "%s | hue %.0f | %.1f cm", readingShown[0], hueShown[0], distShown[0]);
        telemetry.addData("ColorB (pos 2)", "%s | hue %.0f | %.1f cm", readingShown[1], hueShown[1], distShown[1]);
        telemetry.addData("Encoder / target", "%.0f / %d  (error %.0f)", position(), targetTicks, lastError);
        telemetry.addData("Angle", "%.1f deg", position() / TICKS_PER_DEG);
        telemetry.addData("Motor power", "%.2f", lastPower);
        telemetry.update();
    }
}