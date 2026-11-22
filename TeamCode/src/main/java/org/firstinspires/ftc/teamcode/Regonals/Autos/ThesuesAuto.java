package org.firstinspires.ftc.teamcode.Regonals.Autos;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Regonals.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Drive-only auto (from the Pedro Visualizer file "Theuses park"): one straight line, heading held
 * at 90 deg. No turret, shooter, intake or servos are touched. The final pose is saved to
 * PoseStorage every loop so TeleOp can start from where this auto stopped.
 */
@Disabled()
public class ThesuesAuto extends LinearOpMode {

    /** Longest the path may take: after this the auto gives up and holds the pose (5 seconds). */
    public static long PATH_TIMEOUT_MS = 5000;
    /** How long to wait for Pedro to report the new path as busy. */
    public static long PICKUP_TIMEOUT_MS = 500;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(7.0117, 81.3848, 90);
    private final Pose end = poseFactory.of(12.4766, 97.2841, 90);

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        telemetry.addLine("Drive Only ready. Robot on the start pose (7.0, 81.4), facing 90 deg.");
        telemetry.update();

        waitForStart();

        follower.follow(path1());

        // Give the follower a moment to report busy, so the path is never skipped.
        long pickupDeadline = System.currentTimeMillis() + PICKUP_TIMEOUT_MS;
        do {
            loopOnce("starting");
        } while (opModeIsActive() && !follower.isBusy() && System.currentTimeMillis() < pickupDeadline);

        long pathDeadline = System.currentTimeMillis() + PATH_TIMEOUT_MS;
        while (opModeIsActive() && follower.isBusy() && System.currentTimeMillis() < pathDeadline) {
            loopOnce("driving");
        }

        // Done (or timed out): hold where we are until the auto ends.
        follower.hold(follower.pose());
        while (opModeIsActive()) {
            loopOnce("done");
        }
    }

    /** One loop: Pedro, save the pose for TeleOp, telemetry. */
    private void loopOnce(String stage) {
        follower.update();
        PoseStorage.lastPose = follower.pose();
        telemetry.addData("stage", stage);
        telemetry.addData("path completion", "%.2f", follower.completion());
        telemetry.addData("x / y / heading", "%.1f / %.1f / %.0f",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    public Path path1() {
        return line(start, end).linear(start, end);
    }
}