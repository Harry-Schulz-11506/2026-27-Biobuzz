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
 * Two-path auto as a normal LinearOpMode (no Ivy commands), using AutoActions for the mechanisms.
 *
 * Sequence:
 *   1. at the start of path 1 (the start pose): track RED 2, spin up for the distance, wait until
 *      aimed + at speed, then shoot for SHOOT_MS (3 seconds)
 *   2. path 1
 *   3. path 2
 *   4. turret back to 0 deg, everything off
 *
 * The turret must be pointing straight ahead when INIT is pressed: that is where 0 deg is taken.
 */
@Autonomous(name = "1 tip red", group = "Autonomous")
public class Onetipred extends LinearOpMode {

    /** Added to the shot RPM in this auto only (TeleOp and the other autos are not affected). */
    public static double RPM_OFFSET = -,400;
    /** How long to run the transfer for the opening shot. */
    public static long SHOOT_MS = 3000;
    /** How long to wait for "aimed + flywheel at speed" before shooting anyway. */
    public static long READY_TIMEOUT_MS = 2500;
    /** How long to wait for Pedro to report a newly started path as busy. */
    public static long PICKUP_TIMEOUT_MS = 500;
    /** How long to wait for the turret to reach 0 deg (straight ahead) at the end. */
    public static long AIM_TIMEOUT_MS = 1000;

    private Follower follower;
    private AutoActions mech;
    private String stage = "init";

    private final PoseFactory poseFactory = PoseFactory.degrees(); // red side, same as TwoTipExample (add .mirrorX(70.75) for a blue copy)

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56.4109, 12.59, 90);
    private final Pose point2Start = poseFactory.of(56.4109, 12.59, 90);
    private final Pose point2 = poseFactory.of(9.8374, 101.8051, 90);
    private final Pose point2Control1 = poseFactory.of(20.6559, 50.098, 0);

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        mech = new AutoActions(hardwareMap, follower);
        mech.trackGoal(true, false); // RED 2

        telemetry.addLine("1 tip red ready. Point the turret straight ahead, then INIT.");
        telemetry.update();

        waitForStart();

        // ---- 1. Shoot for 3 seconds at the start of path 1, tracking RED 2 ----
        stage = "start shot: RED 2";
        score(SHOOT_MS);

        // ---- 2. Path 1 ----
        stage = "path 1";
        drive(path1());

        // ---- 3. Path 2 ----
        stage = "path 2";
        drive(path2());

        // ---- 4. Turret back to 0 deg, everything off ----
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

    /** Drive one path: Pedro and the mechanisms keep updating until the path is done. */
    private void drive(Path path) {
        follower.follow(path);

        // Give the follower one loop (up to PICKUP_TIMEOUT_MS) to report busy, so a path is never skipped.
        long pickupDeadline = System.currentTimeMillis() + PICKUP_TIMEOUT_MS;
        do {
            loopOnce();
        } while (opModeIsActive() && !follower.isBusy() && System.currentTimeMillis() < pickupDeadline);

        while (opModeIsActive() && follower.isBusy()) {
            loopOnce();
        }
    }

    /**
     * Spin up for the current distance, wait (up to READY_TIMEOUT_MS) until the turret is aimed and the
     * flywheel is at speed, then run the transfer for feedMs. Does not block forever if it times out.
     */
    private void score(long feedMs) {
        mech.stopShoot();
        mech.spinUpForDistance(RPM_OFFSET); // RPM for the distance we are standing at right now, minus 300

        long readyDeadline = System.currentTimeMillis() + READY_TIMEOUT_MS;
        while (opModeIsActive() && !mech.isReadyToShoot() && System.currentTimeMillis() < readyDeadline) {
            loopOnce();
        }

        // Ready: wait-for-speed feed. Timed out: feed anyway, so a flywheel that will not settle cannot stop the feed.
        if (mech.isReadyToShoot()) mech.shoot(); else mech.shootNow();
        long feedDeadline = System.currentTimeMillis() + feedMs;
        while (opModeIsActive() && System.currentTimeMillis() < feedDeadline) {
            loopOnce();
        }
        mech.stopShoot();
    }

    /** Hold the current aim until the turret is within tolerance, bounded by timeoutMs. */
    private void holdAimed(long timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (opModeIsActive() && !mech.isAimed() && System.currentTimeMillis() < deadline) {
            loopOnce();
        }
    }

    /** One loop: Pedro, then the mechanisms (turret + flywheel + gated feed), then telemetry. */
    private void loopOnce() {
        follower.update();
        mech.update();
        telemetry.addData("stage", stage);
        telemetry.addData("path", follower.isBusy() ? "driving" : "done");
        telemetry.addData("x / y", "%.1f / %.1f", follower.pose().x(), follower.pose().y());
        mech.addTelemetry(telemetry);
        telemetry.update();
    }

    // ===================== PATHS =====================

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }
}