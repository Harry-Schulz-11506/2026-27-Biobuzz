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
 * V2 BLUE 1 tip: the V2 RED 1 tip turned 180 degrees about the field centre for the blue alliance (the blue start is on the opposite side), tracks BLUE CELL 2. The path names say what happens:
 *   shootAtstart        shoot at the start pose (BLUE CELL 2), then drive the path
 *   intakethewholepath  intake + transfer run for the whole path
 *   shootatend          drive the path, then shoot at the end of it (BLUE CELL 2)
 *   path4               plain drive (park), then the turret goes back to 0 deg
 * Every path gives up after PATH_TIMEOUT_MS. The stopper is open only while shooting.
 */
@Autonomous(name = "V2 BLUE 1 tip", group = "Autonomous")
public class V2Blue1Tip extends LinearOpMode {

    /** RPM offset for each shot on its own, added to the regression RPM for that shot (this auto only). */
    public static double SHOT_1_RPM_OFFSET = 0;
    public static double SHOT_2_RPM_OFFSET = 0;
    /** Wait (robot stopped, flower down, intake + transfer running) at the end of a flower intake path. */
    public static long FLOWER_WAIT_MS = 1000;
    /** How long to wait for "aimed + flywheel at speed" before shooting anyway. */
    public static long READY_TIMEOUT_MS = 2500;
    /** How long to run the intake + transfer for each shot (3 seconds). */
    public static long FEED_MS = 3000;
    /** Longest any one path may take: after this the auto gives up on it and moves on (5 seconds). */
    public static long PATH_TIMEOUT_MS = 5000;
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

    /** The field is turned 180 degrees about its centre between the alliances (not mirrored). */
    public static double FIELD_SIZE = 144;

    /** Rotates a RED-side pose 180 degrees about the field centre: x -> FIELD - x, y -> FIELD - y, heading + 180. */
    private Pose p(double x, double y, double headingDeg) {
        double h = ((headingDeg + 180) % 360 + 360) % 360;
        if (h > 180) h -= 360;
        return poseFactory.of(FIELD_SIZE - x, FIELD_SIZE - y, h);
    }

    private final Pose start = p(52.8672, 7.1646, 90);
    private final Pose shootAtstart = p(22.583, 6.7601, 180);
    private final Pose shootAtstartControl1 = p(36.7665, 33.7433, 0);
    private final Pose intakethewholepathStart = p(22.583, 6.7601, 180);
    private final Pose intakethewholepath = p(7.9446, 6.7005, 180);
    private final Pose shootatendStart = p(7.9446, 6.7005, 180);
    private final Pose shootatend = p(52.7763, 6.9094, 180);
    private final Pose shootatendControl1 = p(31.6919, 18.4921, 0);
    private final Pose point4Start = p(52.7763, 6.9094, 180);
    private final Pose point4 = p(10.8971, 95.3873, 180);
    private final Pose point4Control1 = p(13.6375, 37.6457, 0);

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        // Turret must be pointing straight ahead when INIT is pressed: that is where 0 deg is taken.
        mech = new AutoActions(hardwareMap, follower);
        mech.trackGoal(false, false); // BLUE CELL 2: the upper blue cell, the 180-degree turn of RED CELL 2

        telemetry.addLine("V2 BLUE 1 tip ready. Point the turret straight ahead, then INIT.");
        telemetry.update();

        waitForStart();

        // ---- 1. shootAtstart: shoot from the start pose, then drive the path ----
        stage = "shoot at start: BLUE CELL 2";
        score(FEED_MS, SHOT_1_RPM_OFFSET);
        stage = "shootAtstart";
        drive(shootAtstartPath());

        // ---- 2. intakethewholepath: intake + transfer on for the whole path ----
        stage = "intakethewholepath";
        mech.intakeInWithTransfer();
        drive(intakethewholepathPath());
        mech.intakeStop();

        // ---- 3. shootatend: drive, then shoot at the end ----
        stage = "shootatend";
        drive(shootatendPath());
        stage = "shoot at end: BLUE CELL 2";
        score(FEED_MS, SHOT_2_RPM_OFFSET);

        // ---- 4. path4, then the turret back to 0 deg ----
        stage = "path4";
        drive(path4Path());
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
        long pathDeadline = System.currentTimeMillis() + PATH_TIMEOUT_MS;
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

    public Path shootAtstartPath() {
        return curve(start, shootAtstartControl1, shootAtstart).linear(start, shootAtstart);
    }

    public Path intakethewholepathPath() {
        return line(intakethewholepathStart, intakethewholepath).linear(intakethewholepathStart, intakethewholepath);
    }

    public Path shootatendPath() {
        return curve(shootatendStart, shootatendControl1, shootatend).linear(shootatendStart, shootatend);
    }

    public Path path4Path() {
        return curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }
}