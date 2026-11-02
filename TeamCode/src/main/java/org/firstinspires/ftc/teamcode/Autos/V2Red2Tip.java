package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.VibeCodeing.AutoActions;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * AutoPath with the mechanisms. The path names say what happens:
 *   shootatstart       shoot at the start pose (RED 2), then drive the path
 *   intakewhilemoving  intake + transfer run while the path is driven
 *   shootatend         drive the path, then shoot at the end of it
 *   flowerintakeatend  the flower goes down (intake + transfer on) halfway along the path BEFORE, the
 *                      robot waits FLOWER_WAIT_MS at the end of this path, and the flower comes back
 *                      up halfway along the path AFTER
 *   path4, path6, path8, path10, path12   plain drives
 *
 * Goal tracked: RED 2 from the start up to halfway along shootatend_2; RED 1 from there to the
 * last shot; the turret goes back to 0 deg at the end.
 * The stopper is open only while shooting, blocking otherwise.
 */
@Autonomous(name = "V2 RED 2 tip", group = "Autonomous")
public class V2Red2Tip extends LinearOpMode {

    /** RPM offset for each shot on its own, added to the regression RPM for that shot (this auto only). */
    public static double SHOT_1_RPM_OFFSET = 0;
    public static double SHOT_2_RPM_OFFSET = 0;
    public static double SHOT_3_RPM_OFFSET = 0;
    public static double SHOT_4_RPM_OFFSET = 0;
    /** Wait (robot stopped, flower down, intake + transfer running) at the end of a flower intake path. */
    public static long FLOWER_WAIT_MS = 1000;
    /** How long to wait for "aimed + flywheel at speed" before shooting anyway. */
    public static long READY_TIMEOUT_MS = 2500;
    /** How long to run the intake + transfer for each shot (3 seconds). */
    public static long FEED_MS = 3000;
    /** Longest any one path may take: after this the auto gives up on it and moves on (5 seconds). */
    public static long PATH_TIMEOUT_MS = 5000;
    /** Timeout for the two paths that go TO the flowers (flowerintakeatend and flowerIntakeatend). */
    public static long FLOWER_PATH_TIMEOUT_MS = 5000;
    /** How long to wait for Pedro report a newly started path as busy. */
    public static long PICKUP_TIMEOUT_MS = 500;
    /** How long to wait for the turret to reach 0 deg (straight ahead) at the end of the auto. */
    public static long AIM_TIMEOUT_MS = 1000;

    private Follower follower;
    private AutoActions mech;
    private String stage = "init";
    private boolean flowerIsDown = false; // flower commanded down: intake + transfer run
    private boolean windowDone = false;   // a timed flower window has run out
    private long intakeEndMs = 0;         // when the current flower intake window ends (0 = none)
    private boolean inShot = false;       // true while score() runs

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.538, 8.2089, 90);
    private final Pose shootatstart = poseFactory.of(23.4185, 7.1779, 180);
    private final Pose shootatstartControl1 = poseFactory.of(60.5936, 28.0883, 0);
    private final Pose intakewhilemovingStart = poseFactory.of(23.4185, 7.1779, 180);
    private final Pose intakewhilemoving = poseFactory.of(8.7992, 7.2673, 180);
    private final Pose intakewhilemovingControl1 = poseFactory.of(15.7143, 13.3491, 0);
    private final Pose shootatendStart = poseFactory.of(8.7992, 7.2673, 180);
    private final Pose shootatend = poseFactory.of(46.6509, 10.2205, 180);
    private final Pose point4Start = poseFactory.of(46.6509, 10.2205, 180);
    private final Pose point4 = poseFactory.of(23.5845, 47.0503, 180);
    private final Pose flowerintakeatendStart = poseFactory.of(23.5845, 47.0503, 180);
    private final Pose flowerintakeatend = poseFactory.of(15.3186, 47.1207, 180); // moved 3 in towards the middle (was x 12.3186)
    private final Pose point6 = poseFactory.of(23.8478, 47.4936, -178.1473);
    private final Pose shootatend_2Start = poseFactory.of(23.8478, 47.4936, 180);
    private final Pose shootatend_2 = poseFactory.of(62.667, 132.5875, 90);
    private final Pose shootatend_2Control1 = poseFactory.of(26.3227, 101.1365, 0);
    private final Pose point8Start = poseFactory.of(62.667, 132.5875, 90);
    private final Pose point8 = poseFactory.of(47.0631, 119.7936, 90);
    private final Pose point8Control1 = poseFactory.of(65.4626, 122.2049, 0);
    private final Pose flowerIntakeatendStart = poseFactory.of(47.0631, 119.7936, 90);
    private final Pose flowerIntakeatend = poseFactory.of(46.9247, 126.3723, 90); // moved 3 in towards the middle (was y 129.3723)
    private final Pose point10 = poseFactory.of(46.7524, 119.9481, 88.9524);
    private final Pose shootatend_3Start = poseFactory.of(46.7524, 119.9481, 90);
    private final Pose shootatend_3 = poseFactory.of(64.7729, 132.7817, 90);
    private final Pose shootatend_3Control1 = poseFactory.of(62.0341, 109.8209, 0);
    private final Pose point12Start = poseFactory.of(64.7729, 132.7817, 90);
    private final Pose point12 = poseFactory.of(9.6544, 119.5809, 180);
    private final Pose point12Control1 = poseFactory.of(36.5772, 116.7967, 0);

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        // Turret must be pointing straight ahead when INIT is pressed: that is where 0 deg is taken.
        mech = new AutoActions(hardwareMap, follower);
        mech.trackGoal(true, false); // RED 2 for the opening shot

        telemetry.addLine("AutoPath ready. Point the turret straight ahead, then INIT.");
        telemetry.update();

        waitForStart();

        // ---- 1. shootatstart: shoot from the start pose, then drive the path ----
        stage = "shoot at start: RED 2";
        score(FEED_MS, SHOT_1_RPM_OFFSET);
        stage = "shootatstart";
        drive(shootatstartPath());

        // ---- 2. intakewhilemoving: intake + transfer on while driving ----
        stage = "intakewhilemoving";
        mech.intakeInWithTransfer();
        drive(intakewhilemovingPath());
        mech.intakeStop();

        // ---- 3. shootatend ----
        stage = "shootatend";
        drive(shootatendPath());
        stage = "shoot at end of shootatend: RED 2";
        score(FEED_MS, SHOT_2_RPM_OFFSET);

        // ---- 4. path4 ----
        stage = "path4";
        drive(path4Path(), at(0.5, () -> flowerDownIntake(0))); // flower down + intake from halfway

        // ---- 5. flowerintakeatend: flower intake at the end, then flower up ----
        stage = "flowerintakeatend";
        driveFor(FLOWER_PATH_TIMEOUT_MS, flowerintakeatendPath());
        stage = "flower intake at the end (flower already down)";
        settle(FLOWER_WAIT_MS);

        // ---- 6. path6: back out ----
        stage = "path6";
        drive(path6Path(), at(0.5, () -> flowerUpStop())); // flower up + intake off at halfway

        // ---- 7. shootatend_2: RED 1 from halfway, shoot at the end ----
        stage = "shootatend_2";
        drive(shootatend_2Path(), at(0.5, () -> mech.trackGoal(true, true))); // RED 1
        stage = "shoot at end of shootatend_2: RED 1";
        score(FEED_MS, SHOT_3_RPM_OFFSET);

        // ---- 8. path8 ----
        stage = "path8";
        drive(path8Path(), at(0.5, () -> flowerDownIntake(0))); // flower down + intake from halfway

        // ---- 9. flowerIntakeatend: flower intake at the end, then flower up ----
        stage = "flowerIntakeatend";
        driveFor(FLOWER_PATH_TIMEOUT_MS, flowerIntakeatendPath());
        stage = "flower intake at the end (flower already down)";
        settle(FLOWER_WAIT_MS);

        // ---- 10. path10: back out ----
        stage = "path10";
        drive(path10Path(), at(0.5, () -> flowerUpStop())); // flower up + intake off at halfway

        // ---- 11. shootatend_3 ----
        stage = "shootatend_3";
        drive(shootatend_3Path());
        stage = "shoot at end of shootatend_3: RED 1";
        score(FEED_MS, SHOT_4_RPM_OFFSET);

        // ---- 12. path12, then the turret back to 0 deg ----
        stage = "path12";
        drive(path12Path());
        stage = "turret to 0 deg";
        mech.aimStraightAhead();
        holdAimed(AIM_TIMEOUT_MS);
        mech.stopAll();
        stage = "done";
        loopOnce();

        while (opModeIsActive()) {
            loopOnce(); // hold the pose, keep telemetry alive until the auto ends
        }
    }

    // ===================== HELPERS =====================

    /** One scheduled mid-path action: run once when the path passes this completion (0..1). */
    private static final class At {
        final double completion;
        final Runnable action;

        At(double completion, Runnable action) {
            this.completion = completion;
            this.action = action;
        }
    }

    /** Schedule an action for a point along a path (0 = start, 0.25 = first quarter, 1 = end). */
    private static At at(double completion, Runnable action) {
        return new At(completion, action);
    }

    /**
     * Drive one path: Pedro updates and the mechanisms keep running until the path is done. Every
     * scheduled action runs once, when the path reaches its completion (0 = start, 0.5 = halfway,
     * 1 = end); use it to switch the tracked goal or move a servo partway along a path.
     */
    private void drive(Path path, At... actions) {
        driveFor(PATH_TIMEOUT_MS, path, actions);
    }

    /** Same as drive(), with its own timeout for this one path. */
    private void driveFor(long timeoutMs, Path path, At... actions) {
        follower.follow(path);

        // Give the follower one loop (up to PICKUP_TIMEOUT_MS) to report busy, so a path is never skipped.
        long pickupDeadline = System.currentTimeMillis() + PICKUP_TIMEOUT_MS;
        do {
            loopOnce();
        } while (opModeIsActive() && !follower.isBusy() && System.currentTimeMillis() < pickupDeadline);

        boolean[] done = new boolean[actions.length];
        boolean[] seenBelow = new boolean[actions.length];
        // completion() reads 1.0 before the path is tracked: wait to see the path below a trigger
        // point before firing that action.
        long pathDeadline = System.currentTimeMillis() + timeoutMs;
        while (opModeIsActive() && follower.isBusy() && System.currentTimeMillis() < pathDeadline) {
            for (int i = 0; i < actions.length; i++) {
                if (done[i]) continue;
                double completion = follower.completion();
                if (completion < actions[i].completion) {
                    seenBelow[i] = true;
                } else if (seenBelow[i]) {
                    actions[i].action.run();
                    done[i] = true;
                }
            }
            loopOnce();
        }
        // Path ended before a trigger point was seen: still apply that action.
        for (int i = 0; i < actions.length; i++) {
            if (!done[i]) actions[i].action.run();
        }
    }

    /**
     * Flower down: it also runs the intake and the transfer for ms, with the stop servo open so
     * artifacts come in. When the time is up they stop and the stop servo goes back to blocking.
     */
    private void flowerDownIntake(long ms) {
        flowerIsDown = true;
        windowDone = false;
        mech.flowerDown();
        mech.intakeInWithTransfer();
        // ms <= 0: run the whole time the flower is down. ms > 0: only for that long.
        intakeEndMs = ms > 0 ? System.currentTimeMillis() + ms : 0;
    }

    /** While the flower is down: keep the intake + transfer running with the stopper open. */
    private void applyFlowerDown() {
        if (!flowerIsDown) return;
        if (!windowDone) {
            mech.intakeInWithTransfer();
        }
    }

    /** After a shot while the flower is still down: restart the intake for the time that is left. */
    private void resumeFlowerIntake() {
        applyFlowerDown();
    }

    /** Flower up: intake and transfer off, stop servo back to blocking. */
    private void flowerUpStop() {
        flowerIsDown = false;
        windowDone = false;
        intakeEndMs = 0;
        mech.flowerUp();
        mech.intakeStop();
        mech.transferStop();
        mech.stopBlocking();
    }

    /**
     * Spin up for the current distance, wait (up to READY_TIMEOUT_MS) until the turret is aimed and the
     * flywheel is at speed, then run the transfer for feedMs. Shooting does not block if it times out.
     */
    private void score(long feedMs, double shotOffset) {
        // The transfer belongs to the shot, not to an earlier intake: zero it so the feed only runs
        // from shoot(), once the flywheel is at speed (GATE_FEED_ON_SPEED). Intake keeps its power.
        inShot = true;
        long shotStart = System.currentTimeMillis();
        mech.stopShoot();
        mech.spinUpForDistance(shotOffset); // RPM for the distance we are at, plus this shot's own offset

        long readyDeadline = System.currentTimeMillis() + READY_TIMEOUT_MS;
        while (opModeIsActive() && !mech.isReadyToShoot() && System.currentTimeMillis() < readyDeadline) {
            loopOnce();
        }

        // Ready: wait-for-speed feed. Timed out: feed anyway, so a flywheel that will not settle within
        // the tolerance cannot stop the intake and transfer from running at all.
        if (mech.isReadyToShoot()) mech.shoot(); else mech.shootNow();
        long feedDeadline = System.currentTimeMillis() + feedMs;
        while (opModeIsActive() && System.currentTimeMillis() < feedDeadline) {
            loopOnce();
        }
        mech.stopShoot();
        mech.intakeStop();
        if (intakeEndMs > 0) intakeEndMs += System.currentTimeMillis() - shotStart; // shooting time does not count
        inShot = false;
    }

    /** Keep the mechanisms alive for ms without driving (for example while a path hands over). */
    private void settle(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            loopOnce();
        }
    }

    /**
     * Hold the current aim until the turret is within tolerance, bounded by timeoutMs. Used at the
     * end of the auto so the turret is back at 0 deg (straight ahead) for TeleOp, which is where
     * its zero was taken at INIT.
     */
    private void holdAimed(long timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (opModeIsActive() && !mech.isAimed() && System.currentTimeMillis() < deadline) {
            loopOnce();
        }
    }

    /** One loop: Pedro, then the mechanisms (turret PID + flywheel PID + gated feed), then telemetry. */
    private void loopOnce() {
        // End of a flower intake window: stop the intake + transfer and hold the artifacts in.
        if (flowerIsDown && !inShot) {
            if (intakeEndMs > 0 && !windowDone && System.currentTimeMillis() >= intakeEndMs) {
                windowDone = true;
                mech.intakeStop();
                mech.transferStop();
                mech.stopBlocking();
            }
            applyFlowerDown();
        }
        follower.update();
        mech.update();
        telemetry.addData("stage", stage);
        telemetry.addData("path", follower.isBusy() ? "driving" : "done");
        telemetry.addData("path completion", "%.2f", follower.completion());
        telemetry.addData("x / y", "%.1f / %.1f", follower.pose().x(), follower.pose().y());
        mech.addTelemetry(telemetry);
        telemetry.update();
    }

    // ===================== PATHS (from the visualiser) =====================

    public Path shootatstartPath() {
        return curve(start, shootatstartControl1, shootatstart).linear(start, shootatstart);
    }

    public Path intakewhilemovingPath() {
        return curve(intakewhilemovingStart, intakewhilemovingControl1, intakewhilemoving).linear(intakewhilemovingStart, intakewhilemoving);
    }

    public Path shootatendPath() {
        return line(shootatendStart, shootatend).linear(shootatendStart, shootatend);
    }

    public Path path4Path() {
        return line(point4Start, point4).linear(point4Start, point4);
    }

    public Path flowerintakeatendPath() {
        return line(flowerintakeatendStart, flowerintakeatend).linear(flowerintakeatendStart, flowerintakeatend);
    }

    public Path path6Path() {
        return line(flowerintakeatend, point6).reverseTangent();
    }

    public Path shootatend_2Path() {
        return curve(shootatend_2Start, shootatend_2Control1, shootatend_2).linear(shootatend_2Start, shootatend_2);
    }

    public Path path8Path() {
        return curve(point8Start, point8Control1, point8).linear(point8Start, point8);
    }

    public Path flowerIntakeatendPath() {
        return line(flowerIntakeatendStart, flowerIntakeatend).linear(flowerIntakeatendStart, flowerIntakeatend);
    }

    public Path path10Path() {
        return line(flowerIntakeatend, point10).reverseTangent();
    }

    public Path shootatend_3Path() {
        return curve(shootatend_3Start, shootatend_3Control1, shootatend_3).linear(shootatend_3Start, shootatend_3);
    }

    public Path path12Path() {
        return curve(point12Start, point12Control1, point12).linear(point12Start, point12);
    }
}