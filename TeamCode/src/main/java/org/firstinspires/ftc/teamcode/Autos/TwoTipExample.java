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
 * EXAMPLE auto: a copy of TwoTip with the mechanism calls added, showing how to drive paths and
 * shoot / transfer / intake with AutoActions.
 *
 * Here "shoot" means the transfer plus the flywheel and the turret: the RPM comes from the
 * ShooterRegression fit for the distance the robot is standing at, the same way MainTeleOp does it.
 * "Intake" means the intake plus the transfer, which is the pairing TeleOp uses too.
 *
 * Sequence in this example:
 *   1. at the start pose: spin up, wait until ready, shoot and run the transfer
 *   2. path 1, then one second of intake (intake + transfer) at the end of it
 *   3. paths 2-3: the flower servo is down, nothing else runs
 *   4. path 4: flower down from halfway, then intake + shoot at the end of it
 *   5. path 5: flower up at the first quarter, then shoot at the end of it
 *   6. path 6: intake WHILE driving, then nothing running
 *   7. path 7, then shoot at the end of it
 *   8. path 8 (park), then the turret goes back to 0 deg and everything off
 *
 * The stop servo opens (position 0, down) for each intake window above and goes back to blocking
 * (0.5) when the window ends, so the artifacts are held in while driving and released while
 * intaking.
 *
 * The flower servo is down (0.6) on paths 2-3, and again from halfway through path 4 to the first
 * quarter of path 5. It is up (0) the rest of the time, including at INIT.
 *
 * Goal the turret tracks:
 *   RED 2 on paths 1 and 2, and from halfway through path 5 to the end of the auto
 *   RED 1 everywhere else (opening shot, paths 3 and 4, and the end-of-path-4 shot)
 *   straight ahead (0 deg) at the end of path 8
 *
 * score() leaves the flywheel target set, so it spins on between shots and only stops in
 * stopAll() at the end of the auto.
 *
 * Copy this file, rename the class + @Autonomous name, and move the mech.* calls to taste.
 */
@Autonomous(name = "TwoTip W/Shooting", group = "Autonomous")
public class TwoTipExample extends LinearOpMode {

    /** How long to wait for "aimed + flywheel at speed" before shooting anyway. */
    public static long READY_TIMEOUT_MS = 2500;
    /** How long to run the transfer to launch the artifact(s). */
    public static long FEED_MS = 1200;
    /** How long to wait for Pedro report a newly started path as busy. */
    public static long PICKUP_TIMEOUT_MS = 500;
    /** How long the end-of-path-1 intake window runs (intake + transfer). */
    public static long INTAKE_MS = 1000;
    /** How long to wait for the turret to reach 0 deg (straight ahead) at the end of the auto. */
    public static long AIM_TIMEOUT_MS = 1000;

    private Follower follower;
    private AutoActions mech;
    private String stage = "init";

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(17.333, 48.5165, 180);
    private final Pose path1Control1 = poseFactory.of(21.7829, 21.6437, 0);
    private final Pose path1Control2 = poseFactory.of(45.4733, 51.2428, 0);
    private final Pose point2 = poseFactory.of(10.0846, 47.8862, 180);
    private final Pose point3 = poseFactory.of(14.8118, 47.8862, 180);
    private final Pose point4 = poseFactory.of(46.9566, 131.0845, 90);
    private final Pose point4Control1 = poseFactory.of(9.1292, 73.4967, 0);
    private final Pose point4Control2 = poseFactory.of(40.4376, 85.7572, 0);
    private final Pose point5 = poseFactory.of(27.0557, 19.0256, 180);
    private final Pose point5Control1 = poseFactory.of(66.1804, 103.982, 0);
    private final Pose point5Control2 = poseFactory.of(58.3018, 5.0266, 0);
    private final Pose point5Control3 = poseFactory.of(59.9031, 71, 0);
    private final Pose point6 = poseFactory.of(8.8797, 5.9465, 180);
    private final Pose point6Control1 = poseFactory.of(64.0546, 6.1949, 0);
    private final Pose point7 = poseFactory.of(49.9343, 13.6559, 180);
    private final Pose point8 = poseFactory.of(8.2383, 97.6236, 180);
    private final Pose point8Control1 = poseFactory.of(5.4989, 53.0557, 0);

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        // Turret must be pointing straight ahead when INIT is pressed: that is where 0 deg is taken.
        mech = new AutoActions(hardwareMap, follower);
        mech.trackGoal(true, true); // RED 1 for the opening shot

        telemetry.addLine("TwoTip example ready. Point the turret straight ahead, then INIT.");
        telemetry.update();

        waitForStart();

        // ---- 1. Shoot and run the transfer from the start pose (RED 1) ----
        stage = "start shot: RED 1";
        score(FEED_MS);

        // ---- 2. Path 1 tracking RED 2 ----
        mech.trackGoal(true, false); // RED 2
        stage = "path 1: RED 2";
        drive(path1());

        // ---- 3. End of path 1: one second of intake (stop open), then nothing on ----
        stage = "end of path 1: intake for 1 s";
        mech.stopDown(); // open, so the intake can take artifacts in
        mech.intakeInWithTransfer();
        settle(INTAKE_MS);
        mech.intakeStop();
        mech.transferStop();
        mech.stopBlocking(); // back to holding the artifacts in

        // ---- 4. Paths 2-3 with the flower down: still RED 2, then back on RED 1 ----
        mech.flowerDown();
        stage = "path 2: RED 2, flower down";
        drive(path2());
        mech.trackGoal(true, true); // RED 1
        stage = "path 3: RED 1, flower down";
        drive(path3());
        mech.flowerUp();

        // ---- 5. Path 4 on RED 1, flower down from halfway, then intake + shoot ----
        stage = "path 4: RED 1, flower down from halfway";
        drive(path4(), at(0.5, () -> mech.flowerDown()));
        stage = "end of path 4: intake + shoot: RED 1";
        mech.stopDown(); // open for the intake, the transfer feeds through it
        mech.intakeInWithTransfer();
        score(FEED_MS);
        mech.intakeStop();
        mech.transferStop();
        mech.stopBlocking();

        // ---- 6. Path 5: flower up at the first quarter, RED 2 from halfway, shoot at the end ----
        stage = "path 5: flower up at 1/4, RED 2 from halfway";
        drive(path5(), at(0.25, () -> mech.flowerUp()), at(0.5, () -> mech.trackGoal(true, false)));
        stage = "end of path 5: shoot: RED 2";
        score(FEED_MS);

        // ---- 7. Path 6 tracking RED 2 with the intake running the whole way ----
        stage = "path 6: RED 2, intake while driving";
        mech.stopDown(); // open for the intake
        mech.intakeInWithTransfer();
        drive(path6());
        mech.intakeStop();
        mech.transferStop();
        mech.stopBlocking();

        // ---- 8. Path 7 tracking RED 2, then shoot at the end of it ----
        stage = "path 7: RED 2";
        drive(path7());
        stage = "end of path 7: shoot: RED 2";
        score(FEED_MS);

        // ---- 9. Park on RED 2, then the turret goes back to 0 deg at the end of path 8 ----
        stage = "path 8: RED 2";
        drive(path8());
        stage = "end of path 8: turret to 0 deg";
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
        while (opModeIsActive() && follower.isBusy()) {
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
     * Spin up for the current distance, wait (up to READY_TIMEOUT_MS) until the turret is aimed and the
     * flywheel is at speed, then run the transfer for feedMs. Shooting does not block if it times out.
     */
    private void score(long feedMs) {
        // The transfer belongs to the shot, not to an earlier intake: zero it so the feed only runs
        // from shoot(), once the flywheel is at speed (GATE_FEED_ON_SPEED). Intake keeps its power.
        mech.stopShoot();
        mech.spinUpForDistance(); // RPM for the distance we are standing at right now

        long readyDeadline = System.currentTimeMillis() + READY_TIMEOUT_MS;
        while (opModeIsActive() && !mech.isReadyToShoot() && System.currentTimeMillis() < readyDeadline) {
            loopOnce();
        }

        mech.shoot(); // transfer only runs once the flywheel is at speed (GATE_FEED_ON_SPEED)
        long feedDeadline = System.currentTimeMillis() + feedMs;
        while (opModeIsActive() && System.currentTimeMillis() < feedDeadline) {
            loopOnce();
        }
        mech.stopShoot();
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
        follower.update();
        mech.update();
        telemetry.addData("stage", stage);
        telemetry.addData("path", follower.isBusy() ? "driving" : "done");
        telemetry.addData("path completion", "%.2f", follower.completion());
        telemetry.addData("x / y", "%.1f / %.1f", follower.pose().x(), follower.pose().y());
        mech.addTelemetry(telemetry);
        telemetry.update();
    }

    // ===================== PATHS (same as TwoTip) =====================

    public Path path1() {
        return curve(start, path1Control1, path1Control2, path1).linear(start, path1);
    }

    public Path path2() {
        return line(path1, point2).linear(path1, point2);
    }

    public Path path3() {
        return line(point2, point3).linear(point2, point3);
    }

    public Path path4() {
        return curve(point3, point4Control1, point4Control2, point4).linear(point3, point4);
    }

    public Path path5() {
        return curve(point4, point5Control1, point5Control2, point5Control3, point5).linear(point4, point5);
    }

    public Path path6() {
        return curve(point5, point6Control1, point6).linear(point5, point6);
    }

    public Path path7() {
        return line(point6, point7).linear(point6, point7);
    }

    public Path path8() {
        return curve(point7, point8Control1, point8).linear(point7, point8);
    }
}
