package org.firstinspires.ftc.teamcode.NATS;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * Relocalization points in all 4 corners of the field (Pedro coordinates, 144 x 144 in).
 *
 * TeleOp (gamepad 2): hold START and press (bumpers = bottom corners, triggers = top corners)
 *     left bumper  = bottom-left  (x 0,   y 0)     right bumper  = bottom-right (x 144, y 0)
 *     left trigger = top-left     (x 0,   y 144)   right trigger = top-right    (x 144, y 144)
 * A and B are NOT used on purpose: START + A / START + B re-pair the controllers on the Driver Station.
 * Drive the robot square into the corner first, then press. The pose is the robot CENTRE: the robot's
 * half length / half width (RobotConfig.ROBOT_LENGTH / ROBOT_WIDTH) are added from the walls, using the
 * corner heading in RobotConfig (CORNER_*_HEADING).
 */
public class FieldCorners {

    public enum Corner { BOTTOM_LEFT, BOTTOM_RIGHT, TOP_LEFT, TOP_RIGHT }

    private static boolean prevLB, prevRB, prevLT, prevRT;
    private static final double TRIGGER_ON = 0.5;

    /** The robot-centre pose for a corner, with the robot squared into it. */
    public static Pose pose(Corner corner) {
        double headingDeg;
        boolean right, top;
        switch (corner) {
            case BOTTOM_LEFT:  headingDeg = RobotConfig.CORNER_BL_HEADING; right = false; top = false; break;
            case BOTTOM_RIGHT: headingDeg = RobotConfig.CORNER_BR_HEADING; right = true;  top = false; break;
            case TOP_LEFT:     headingDeg = RobotConfig.CORNER_TL_HEADING; right = false; top = true;  break;
            default:           headingDeg = RobotConfig.CORNER_TR_HEADING; right = true;  top = true;  break;
        }
        double h = Math.toRadians(headingDeg);
        double halfL = RobotConfig.ROBOT_LENGTH / 2.0;
        double halfW = RobotConfig.ROBOT_WIDTH / 2.0;
        // Robot facing 0/180 deg: its length is along X. Facing 90/270 deg: its length is along Y.
        double offX = Math.abs(Math.cos(h)) * halfL + Math.abs(Math.sin(h)) * halfW;
        double offY = Math.abs(Math.sin(h)) * halfL + Math.abs(Math.cos(h)) * halfW;
        double x = right ? RobotConfig.FIELD_SIZE - offX : offX;
        double y = top ? RobotConfig.FIELD_SIZE - offY : offY;
        return new Pose(x, y, h);
    }

    /** Put the follower (and the pose TeleOp/auto share) at a corner. */
    public static void relocalize(Follower follower, Corner corner) {
        Pose p = pose(corner);
        if (follower != null) follower.setPose(p);
        PoseStorage.lastPose = p;
    }

    /**
     * Call every TeleOp loop with gamepad 2. Returns true while START is held: the caller should then
     * ignore gamepad 2's bumpers/triggers (the corner buttons) and its other buttons.
     */
    public static boolean update(Gamepad gamepad2, Follower follower) {
        if (gamepad2 == null) return false;
        boolean start = gamepad2.start;
        boolean lb = gamepad2.left_bumper, rb = gamepad2.right_bumper;
        boolean lt = gamepad2.left_trigger > TRIGGER_ON, rt = gamepad2.right_trigger > TRIGGER_ON;
        if (start) {
            if (lb && !prevLB) relocalize(follower, Corner.BOTTOM_LEFT);
            if (rb && !prevRB) relocalize(follower, Corner.BOTTOM_RIGHT);
            if (lt && !prevLT) relocalize(follower, Corner.TOP_LEFT);
            if (rt && !prevRT) relocalize(follower, Corner.TOP_RIGHT);
        }
        prevLB = lb; prevRB = rb; prevLT = lt; prevRT = rt;
        return start;
    }
}
