package org.firstinspires.ftc.teamcode.VibeCodeing;

import com.pedropathing.math.Pose;

/**
 * Carries the robot's pose from the auto into TeleOp. AutoActions saves the follower's pose every loop;
 * the Turret (TeleOp) starts from it when USE_AUTO_POSE is on. It lives in memory only, so it is empty
 * after the robot app restarts, and TeleOp then falls back to START_X / START_Y / START_HEADING.
 */
public class PoseStorage {
    public static Pose lastPose = null;

    public static void clear() { lastPose = null; }
}