package org.firstinspires.ftc.teamcode.NATS.Autos;

import static com.pedropathing.api.Paths.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NATS.AutoActions;
import org.firstinspires.ftc.teamcode.NATS.RobotConfig;
import org.firstinspires.ftc.teamcode.NATS.TipDetector;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * TWO TIP AUTO - RED side, built for a 16 x 14 in robot.
 *
 *   1. shoot the preload (from the start pose)
 *   2. tip check: if that hive's tip has already fallen, start at the other one
 *   3. ball line, heading 180 the whole way, intake running
 *   4. drive to the hive, heading 0 (facing the way it drove in)
 *   5. flower ONCE: line up at the flower's heading (90 for flower 1, 180 for flower 2),
 *      drive straight in, intake, back straight out, then to the hive
 *   6. shoot the first hive: RobotConfig.AUTO_FIRST_HIVE_MAX_SHOTS (1), tip watched after every shot
 *   7. drive the other hive and shoot it: RobotConfig.AUTO_OTHER_HIVE_MAX_SHOTS (3)
 *   8. park
 *
 * A "tip" is a LOWER AprilTag: while the camera sees a tag, TipDetector remembers the highest height
 * since resetTip(), and the tip counts once the tag drops RobotConfig.TIP_DROP_IN below it. The reset
 * happens on arrival at a hive, so the shots at that hive accumulate instead of re-baselining.
 *
 * FIELD POINTS (red side, inches): start (57, 6.5), ball line (18, 6.5) -> (7, 6.5),
 * hive 1 (54, 10) = lower tip = CELL 2, hive 2 (54, 130) = upper tip = CELL 1,
 * flower 1 (12, 47), flower 2 (47, 130), park (12, 120).
 *
 * EVERYTHING HERE ASSUMES THE 16 x 14 BODY: routes hold the robot's centre at least half a width
 * (7 in) plus margin away from walls and obstacles, which is why the cross paths and the park run up
 * the left side at x ~ 30 / 24 instead of cutting across the hive block (47-71, 46-90) or the
 * wall box (2-13.5, 94-118). tools/autopath-preview/check.py audits the clearance; run it after any
 * path change.
 *
 * The turret must be pointing straight ahead when INIT is pressed: that is where Turret takes its
 * 0 deg, and AutoActions carries that zero into TeleOp through PoseStorage.
 */
@Config
@Autonomous(name = "2 Tip Auto", group = "Auto")
public class TwoTipAuto extends LinearOpMode {

    // ===================== STATE =====================
    private Follower follower;
    private AutoActions mech;
    private TipDetector tips;
    private String stage = "init";

    private boolean firstHive1 = true;    // hive the preload went to (or the other one, if its tip had fallen)
    private boolean tippedAtHive = false; // did the first hive's tip fall?
    private boolean tippedOther = false;  // did the second hive's tip fall?
    private int finalHive = 1;            // where the auto ends up (for the park path)
    private long runStartMs = 0;          // set at START, for RobotConfig.AUTO_BUDGET_MS

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(START);
        follower.update();

        // Turret must be pointing straight ahead here: this is where 0 deg is taken.
        mech = new AutoActions(hardwareMap, follower);
        tips = mech.getTipDetector();
        mech.trackGoal(RobotConfig.AUTO_RED_ALLIANCE, RobotConfig.AUTO_PRELOAD_CELL_IS_1);

        telemetry.addLine("2 Tip Auto (RED) ready. 16 x 14 robot.");
        telemetry.addData("preload / first hive", "CELL %d / HIVE %s", RobotConfig.AUTO_PRELOAD_CELL_IS_1 ? 1 : 2,
                cellIsHive1(RobotConfig.AUTO_PRELOAD_CELL_IS_1) ? "1" : "2");
        telemetry.addData("tip", "counts when the tag drops %.1f in", RobotConfig.TIP_DROP_IN);
        telemetry.addLine("HuskyLens sees the tip: " + (tips.isConnected() ? "yes" : "NOT CONNECTED"));
        telemetry.addLine("Point the turret straight ahead, then press START.");
        telemetry.update();

        waitForStart();
        runStartMs = System.currentTimeMillis();

        // ---- 1. Shoot the preload ----
        firstHive1 = cellIsHive1(RobotConfig.AUTO_PRELOAD_CELL_IS_1);
        finalHive = firstHive1 ? 1 : 2;
        stage = "1. preload shot: CELL " + (RobotConfig.AUTO_PRELOAD_CELL_IS_1 ? 1 : 2);
        score(RobotConfig.AUTO_PRELOAD_FEED_MS);

        // ---- 2. Has that hive's tip already fallen? If so, start at the other hive ----
        stage = "2. tip check: HIVE " + (firstHive1 ? 1 : 2);
        tips.resetTip();
        boolean alreadyTipped = awaitTip(RobotConfig.AUTO_TIP_CHOICE_MS);
        if (alreadyTipped) {
            firstHive1 = !firstHive1;
            finalHive = firstHive1 ? 1 : 2;
        }
        stage = "2. tip check: " + (alreadyTipped ? "already tipped -> HIVE " + finalHive
                                                : "not tipped -> stay on HIVE " + finalHive);

        // ---- 3. Ball line, heading 180, intake running the whole way ----
        stage = "3. ball line (heading 180): intake";
        mech.pivotDown();
        mech.intakeInWithTransfer();
        drive(pathIntake());
        settle(RobotConfig.AUTO_INTAKE_MS);
        mech.intakeStop();
        mech.pivotUp();

        // ---- 4. Drive to the hive ----
        stage = "4. path: HIVE " + (firstHive1 ? 1 : 2);
        if (firstHive1) drive(pathToHive1());
        else drive(pathToHive2());

        // ---- 5. The one flower visit: line up, drive in, back out, then to the hive ----
        if (RobotConfig.AUTO_FLOWER_TRIP) flowerTrip(firstHive1);

        // ---- 6. First hive: one shot normally tips it ----
        mech.trackGoal(RobotConfig.AUTO_RED_ALLIANCE, mechCell(firstHive1));
        tips.resetTip();
        tippedAtHive = shootUntilTipped("HIVE " + (firstHive1 ? 1 : 2), RobotConfig.AUTO_FIRST_HIVE_MAX_SHOTS);

        // ---- 7. Score the other hive: it normally needs 2-3 shots ----
        finalHive = firstHive1 ? 2 : 1;
        stage = "7. path: HIVE " + finalHive;
        drive(firstHive1 ? pathHive1ToHive2() : pathHive2ToHive1());

        mech.trackGoal(RobotConfig.AUTO_RED_ALLIANCE, mechCell(finalHive == 1));
        tips.resetTip();
        tippedOther = shootUntilTipped("HIVE " + finalHive, RobotConfig.AUTO_OTHER_HIVE_MAX_SHOTS);

        // ---- 8. Park, turret back to 0 deg, everything off ----
        stage = "8. park from HIVE " + finalHive;
        if (finalHive == 1) drive(pathParkFromHive1());
        else drive(pathParkFromHive2());

        stage = "turret to 0 deg";
        mech.aimStraightAhead();
        holdAimed(RobotConfig.AUTO_AIM_TIMEOUT_MS);
        mech.stopAll();
        stage = "done";
        loopOnce();

        while (opModeIsActive()) {
            loopOnce(); // hold the pose until the auto ends
        }
    }

    // ===================== HIVE / CELL MAPPING =====================

    /** Which cell a hive spot shoots: the bottom spot is CELL 2, the top spot is CELL 1. */
    private boolean mechCell(boolean hive1) { return hive1 ? RobotConfig.AUTO_HIVE1_IS_CELL1 : RobotConfig.AUTO_HIVE2_IS_CELL1; }

    /** The reverse: is this cell's hive spot HIVE 1? */
    private boolean cellIsHive1(boolean cell1) { return cell1 == RobotConfig.AUTO_HIVE1_IS_CELL1; }

    // ===================== TIP =====================

    /**
     * Watch the tip for up to timeoutMs and return true as soon as the tag has dropped. Runs the
     * normal loop, so the detector is read every loop - which is "whenever the camera can see it".
     */
    private boolean awaitTip(long timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (opModeIsActive() && System.currentTimeMillis() < deadline) {
            loopOnce();
            if (tips.hasTipped()) return true;
        }
        return tips.hasTipped();
    }

    // ===================== ACTIONS =====================

    /**
     * Shoot one hive up to maxShots times, watching its tip after every shot and stopping the moment
     * the tag has dropped. Returns true if it tipped.
     *
     * tips.resetTip() belongs to the arrival, not to each shot: the drop is measured against the
     * height the hive had when you got there, so shots accumulate instead of re-baselining.
     */
    private boolean shootUntilTipped(String hiveName, int maxShots) {
        for (int shot = 1; shot <= maxShots; shot++) {
            if (budgetSpent()) {
                stage = hiveName + ": out of auto time, stopping at " + (shot - 1) + " shot(s)";
                return tips.hasTipped();
            }
            stage = hiveName + ": shot " + shot + " / " + maxShots;
            score(RobotConfig.AUTO_FEED_MS);
            stage = hiveName + ": watching the tip (shot " + shot + ")";
            if (awaitTip(RobotConfig.AUTO_TIP_WATCH_MS)) return true;
        }
        return tips.hasTipped();
    }

    /** True once RobotConfig.AUTO_BUDGET_MS has passed since START (0 = never). */
    private boolean budgetSpent() {
        return RobotConfig.AUTO_BUDGET_MS > 0 && System.currentTimeMillis() - runStartMs > RobotConfig.AUTO_BUDGET_MS;
    }

    /** Spin up for the distance we are standing at, wait until ready, feed for feedMs. */
    private void score(long feedMs) {
        mech.stopShoot();
        mech.spinUpForDistance(RobotConfig.AUTO_RPM_OFFSET);

        long readyDeadline = System.currentTimeMillis() + RobotConfig.AUTO_READY_TIMEOUT_MS;
        while (opModeIsActive() && !mech.isReadyToShoot() && System.currentTimeMillis() < readyDeadline) {
            loopOnce();
        }

        // Ready: wait-for-speed feed. Timed out: feed anyway, so a flywheel that will not settle
        // within tolerance cannot stop the whole auto from scoring.
        if (mech.isReadyToShoot()) mech.shoot();
        else mech.shootNow();

        long feedDeadline = System.currentTimeMillis() + feedMs;
        while (opModeIsActive() && System.currentTimeMillis() < feedDeadline) {
            loopOnce();
        }
        mech.stopShoot();
    }

    /**
     * The one flower visit: line up at the flower's heading, drive straight in with the intake on,
     * hold for RobotConfig.AUTO_FLOWER_INTAKE_MS, back straight out to the line-up pose, then drive to the hive.
     */
    private void flowerTrip(boolean hive1) {
        int n = hive1 ? 1 : 2;

        stage = "5. flower " + n + ": line up";
        drive(hive1 ? pathToFlower1() : pathToFlower2());

        stage = "5. flower " + n + ": drive in, intake";
        mech.flowerDown();
        mech.intakeInWithTransfer();
        drive(hive1 ? pathIntoFlower1() : pathIntoFlower2());
        settle(RobotConfig.AUTO_FLOWER_INTAKE_MS);
        mech.intakeStop();
        mech.flowerUp();

        stage = "5. flower " + n + ": back out";
        drive(hive1 ? pathBackFromFlower1() : pathBackFromFlower2());

        stage = "5. flower " + n + ": back to the hive";
        drive(hive1 ? pathBackToHive1() : pathBackToHive2());
    }

    /** Keep the mechanisms alive for ms without driving. */
    private void settle(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            loopOnce();
        }
    }

    /**
     * Drive one path, keeping Pedro, the turret PID and the flywheel PID running. Pedro reports
     * completion() = 1 before a path is being tracked, so a path is only considered done once it
     * has been seen busy (hence the pickup wait).
     */
    private void drive(Path path) {
        follower.follow(path);

        long pickupDeadline = System.currentTimeMillis() + RobotConfig.AUTO_PICKUP_TIMEOUT_MS;
        do {
            loopOnce();
        } while (opModeIsActive() && !follower.isBusy() && System.currentTimeMillis() < pickupDeadline);

        while (opModeIsActive() && follower.isBusy()) {
            loopOnce();
        }
    }

    /** Hold the current aim until the turret is within tolerance, bounded by timeoutMs. */
    private void holdAimed(long timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (opModeIsActive() && !mech.isAimed() && System.currentTimeMillis() < deadline) {
            loopOnce();
        }
    }

    /** One loop: Pedro, then the mechanisms (turret + flywheel + gated feed + vision), then telemetry. */
    private void loopOnce() {
        follower.update();
        mech.update();
        telemetry.addData("stage", stage);
        telemetry.addData("path", follower.isBusy() ? "driving" : "done");
        telemetry.addData("x / y / heading", "%.1f / %.1f / %.0f",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.addData("tip: drop / tipped", "%.1f in / %s", tips.getDropIn(), tips.hasTipped());
        telemetry.addData("hive: first / final", "HIVE %d / HIVE %d", firstHive1 ? 1 : 2, finalHive);
        telemetry.addData("tipped: first / other", "%s / %s", tippedAtHive, tippedOther);
        telemetry.addData("shots allowed: first / other", "%d / %d", RobotConfig.AUTO_FIRST_HIVE_MAX_SHOTS, RobotConfig.AUTO_OTHER_HIVE_MAX_SHOTS);
        mech.addTelemetry(telemetry);
        telemetry.update();
    }

    // ===================== PATHS (RED side, 16 x 14 robot) =====================
    // Headings are fixed by the mechanism, not by taste:
    //   ball line 180, flower 1 90, flower 2 180. Every other heading is the direction of travel,
    //   so the robot never pivots without reason.
    // The *_C* poses are route controls chosen to keep a 16 x 14 body clear of the walls, the wall
    // box (2-13.5, 94-118) and the hive block (47-71, 46-90). check.py verifies the clearance.

    private final PoseFactory pf = PoseFactory.degrees();

    /** Start (57, 6.5) heading 180: the ball line runs straight out of here. */
    private final Pose START = pf.of(57, 6.5, 180);

    /** End of the ball line (7, 6.5), still heading 180. */
    private final Pose BALL_END = pf.of(7, 6.5, 180);

    /** Shoot HIVE 1: bottom spot (54, 10), heading 0 (facing +x, the way the robot drives in). */
    private final Pose HIVE1 = pf.of(54, 10, 0);

    /** Shoot HIVE 2: top spot (54, 130), heading 0 (facing +x, the way the robot drives in). */
    private final Pose HIVE2 = pf.of(54, 130, 0);

    /** Flower 1 (12, 47) at heading 90, with a line-up pose 12 in straight out of it. */
    private final Pose FLOWER1 = pf.of(12, 47, 90);
    private final Pose FLOWER1_APPROACH = pf.of(12, 35, 90);

    /** Flower 2 (47, 130) at heading 180, with a line-up pose 12 in straight out of it (towards +x). */
    private final Pose FLOWER2 = pf.of(47, 130, 180);
    private final Pose FLOWER2_APPROACH = pf.of(59, 130, 180);

    /**
     * Park, heading 180. You asked for (12, 120), but a 16 x 14 body centred there spans y 113-127 and
     * x 4-20, which is 6 in inside the wall box (2-13.5, 94-118). Moved to (24, 128): the body then
     * sits at x 16-32, y 121-135, clear of the box and 9 in from the top wall.
     */
    private final Pose PARK = pf.of(24, 128, 180);
    private final Pose PARK_C1 = pf.of(30, 26, 0);
    private final Pose PARK_C2 = pf.of(30, 118, 0);
    private final Pose PARK_FROM_HIVE2_C1 = pf.of(38, 132, 180);

    /** Route control points: one named constant per leg, so the preview can parse them. */
    private final Pose TO_HIVE1_C1 = pf.of(30, 8, 180);
    private final Pose TO_HIVE2_C1 = pf.of(20, 60, 180);
    private final Pose TO_HIVE2_C2 = pf.of(32, 126, 0);
    private final Pose TO_FLOWER1_C1 = pf.of(30, 16, 0);
    private final Pose BACK_TO_HIVE1_C1 = pf.of(34, 18, 0);
    private final Pose HIVE_CROSS_C1 = pf.of(30, 26, 0);
    private final Pose HIVE_CROSS_C2 = pf.of(30, 108, 0);

    /** Step 3: straight down the ball line at heading 180, (57, 6.5) -> (7, 6.5). */
    public Path pathIntake() {
        return line(START, BALL_END).linear(START, BALL_END);
    }

    /** Step 4: ball line -> bottom hive spot, along the bottom edge. */
    public Path pathToHive1() {
        return curve(BALL_END, TO_HIVE1_C1, HIVE1).linear(BALL_END, HIVE1);
    }

    /** Step 4: ball line -> top hive spot, up the left side then right along the top. */
    public Path pathToHive2() {
        return curve(BALL_END, TO_HIVE2_C1, TO_HIVE2_C2, HIVE2).linear(BALL_END, HIVE2);
    }

    /** Step 5 for HIVE 1: bottom hive spot -> flower 1 line-up pose. */
    public Path pathToFlower1() {
        return curve(HIVE1, TO_FLOWER1_C1, FLOWER1_APPROACH).linear(HIVE1, FLOWER1_APPROACH);
    }

    /** Step 5 for HIVE 1: drive straight in to flower 1 (12 in, heading 90). */
    public Path pathIntoFlower1() {
        return line(FLOWER1_APPROACH, FLOWER1).linear(FLOWER1_APPROACH, FLOWER1);
    }

    /** Step 5 for HIVE 1: back straight out of flower 1. */
    public Path pathBackFromFlower1() {
        return line(FLOWER1, FLOWER1_APPROACH).linear(FLOWER1, FLOWER1_APPROACH);
    }

    /** Step 5 for HIVE 1: line-up pose -> bottom hive spot. */
    public Path pathBackToHive1() {
        return curve(FLOWER1_APPROACH, BACK_TO_HIVE1_C1, HIVE1).linear(FLOWER1_APPROACH, HIVE1);
    }

    /** Step 5 for HIVE 2: top hive spot -> flower 2 line-up pose. */
    public Path pathToFlower2() {
        return line(HIVE2, FLOWER2_APPROACH).linear(HIVE2, FLOWER2_APPROACH);
    }

    /** Step 5 for HIVE 2: drive straight in to flower 2 (12 in, heading 180). */
    public Path pathIntoFlower2() {
        return line(FLOWER2_APPROACH, FLOWER2).linear(FLOWER2_APPROACH, FLOWER2);
    }

    /** Step 5 for HIVE 2: back straight out of flower 2. */
    public Path pathBackFromFlower2() {
        return line(FLOWER2, FLOWER2_APPROACH).linear(FLOWER2, FLOWER2_APPROACH);
    }

    /** Step 5 for HIVE 2: line-up pose -> top hive spot. */
    public Path pathBackToHive2() {
        return line(FLOWER2_APPROACH, HIVE2).linear(FLOWER2_APPROACH, HIVE2);
    }

    /** Step 7: bottom hive spot -> top hive spot, up the left of the hive block. */
    public Path pathHive1ToHive2() {
        return curve(HIVE1, HIVE_CROSS_C1, HIVE_CROSS_C2, HIVE2).linear(HIVE1, HIVE2);
    }

    /** Step 7: top hive spot -> bottom hive spot, back down the left of the hive block. */
    public Path pathHive2ToHive1() {
        return curve(HIVE2, HIVE_CROSS_C2, HIVE_CROSS_C1, HIVE1).linear(HIVE2, HIVE1);
    }

    /** Park from the bottom hive spot: left, up the middle-left lane, then to the park pose. */
    public Path pathParkFromHive1() {
        return curve(HIVE1, PARK_C1, PARK_C2, PARK).linear(HIVE1, PARK);
    }

    /** Park from the top hive spot: across the top, above the wall box. */
    public Path pathParkFromHive2() {
        return curve(HIVE2, PARK_FROM_HIVE2_C1, PARK).linear(HIVE2, PARK);
    }
    // ===================== END PATHS =====================
}
