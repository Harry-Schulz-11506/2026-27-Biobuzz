package org.firstinspires.ftc.teamcode.VibeCodeing;

import com.pedropathing.math.Pose;

/**
 * Carries the robot's pose from the auto into TeleOp. AutoActions saves the follower's pose every loop;
 * the Turret (TeleOp) starts from it when USE_AUTO_POSE is on. It lives in memory only, so it is empty
 * after the robot app restarts, and TeleOp then falls back to START_X / START_Y / START_HEADING.
 */
public class PoseStorage {
    public static Pose lastPose = null;

    /**
     * Encoder reading that counts as turret 0 deg. The turret encoder is not reset between OpModes, so
     * keeping this number lets TeleOp carry on from the angle the turret finished the auto at.
     */
    public static Integer turretZeroTicks = null;

    public static void clear() { lastPose = null; turretZeroTicks = null; }
}