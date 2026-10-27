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
 * Sequence in this example:
 *   1. at the start pose: spin up, wait until ready, shoot and run the transfer
 *   2. path 1
 *   3. end of path 1: intake in + transfer in
 *   4. path 3 (paths 2-3 keep intaking), end of path 3: intake + shoot + transfer
 *   5. paths 4-5 keep the flywheel spinning
 *   6. path 6: intake and transfer WHILE driving
 *   7. path 7, end of path 7: shoot + intake + transfer
 *   8. path 8 (park), then everything off
 *
 * Goal the turret tracks:
 *   RED 2 on paths 1 and 2, and from halfway through path 5 to halfway through path 8
 *   RED 1 everywhere else (opening shot, paths 3, 4, first half of path 5)
 *   straight ahead (0 deg) from halfway through path 8 to the end
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

        // ---- 2-3. Path 1 tracking RED 2, then start intaking ----
        mech.trackGoal(true, false); // RED 2
        stage = "path 1: RED 2";
        drive(path1());
        stage = "end of path 1: intake + transfer";
        mech.intakeIn();
        mech.transferIn();
        settle(200);

        // ---- 4. Path 2 still RED 2, then path 3 back on RED 1 ----
        stage = "path 2: RED 2";
        drive(path2());
        mech.trackGoal(true, true); // RED 1
        stage = "path 3: RED 1";
        drive(path3());
        stage = "end of path 3: intake + shoot + transfer";
        mech.intakeIn();
        score(FEED_MS);

        // ---- 5. Path 4 on RED 1, path 5 flips to RED 2 halfway ----
        stage = "path 4: RED 1";
        drive(path4());
        stage = "path 5: RED 1, then RED 2 from halfway";
        drive(path5(), 0.5, () -> mech.trackGoal(true, false));

        // ---- 6. Path 6 tracking RED 2 with intake and transfer running the whole time ----
        stage = "path 6: RED 2, intake + transfer while driving";
        mech.intakeIn();
        mech.transferIn();
        drive(path6());

        // ---- 7. Path 7 tracking RED 2, then shoot + intake + transfer ----
        stage = "path 7: RED 2";
        drive(path7());
        stage = "end of path 7: shoot + intake + transfer";
        mech.intakeIn();
        score(FEED_MS);

        // ---- 8. Park on RED 2, then point the turret straight ahead (0 deg) halfway through ----
        stage = "path 8: RED 2, straight ahead from halfway";
        drive(path8(), 0.5, () -> mech.aimStraightAhead());
        mech.stopAll();
        stage = "done";
        loopOnce();

        while (opModeIsActive()) {
            loopOnce(); // hold the pose, keep telemetry alive until the auto ends
        }
    }

    // ===================== HELPERS =====================

    /** Drive one path: Pedro updates and the mechanisms keep running until the path is done. */
    private void drive(Path path) {
        drive(path, 2.0, null);
    }

    /**
     * Drive one path, running midAction once when the path is midAt complete (0 = start, 0.5 = halfway,
     * 1 = end). Used to switch the tracked goal partway along a path.
     */
    private void drive(Path path, double midAt, Runnable midAction) {
        follower.follow(path);

        // Give the follower one loop (up to PICKUP_TIMEOUT_MS) to report busy, so a path is never skipped.
        long pickupDeadline = System.currentTimeMillis() + PICKUP_TIMEOUT_MS;
        do {
            loopOnce();
        } while (opModeIsActive() && !follower.isBusy() && System.currentTimeMillis() < pickupDeadline);

        boolean midDone = (midAction == null);
        boolean seenBelowMid = false; // completion() reads 1.0 before the path is tracked; wait for the start
        while (opModeIsActive() && follower.isBusy()) {
            if (!midDone) {
                double completion = follower.completion();
                if (completion < midAt) {
                    seenBelowMid = true;
                } else if (seenBelowMid) {
                    midAction.run();
                    midDone = true;
                }
            }
            loopOnce();
        }
        if (!midDone) midAction.run(); // path ended before the trigger was seen: still apply it
    }

    /**
     * Spin up for the current distance, wait (up to READY_TIMEOUT_MS) until the turret is aimed and the
     * flywheel is at speed, then run the transfer for feedMs. Shooting does not block if it times out.
     */
    private void score(long feedMs) {
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
