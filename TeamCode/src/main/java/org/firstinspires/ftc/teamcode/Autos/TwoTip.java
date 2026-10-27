package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class TwoTip extends LinearOpMode {

    private Follower follower;

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

    private Path[] paths;
    private int pathIndex = 0;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        // Build all paths up front
        paths = new Path[] {
                path1(), path2(), path3(), path4(),
                path5(), path6(), path7(), path8()
        };

        waitForStart();

        // Start the first path
        pathIndex = 0;
        follower.follow(paths[pathIndex]);

        while (opModeIsActive()) {
            follower.update();

            // When the current path finishes, start the next one
            if (!follower.isBusy() && pathIndex < paths.length - 1) {
                pathIndex++;
                follower.follow(paths[pathIndex]);
            }

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Path number", pathIndex + 1);
            telemetry.addData("Busy", follower.isBusy());
            telemetry.update();
        }
    }

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