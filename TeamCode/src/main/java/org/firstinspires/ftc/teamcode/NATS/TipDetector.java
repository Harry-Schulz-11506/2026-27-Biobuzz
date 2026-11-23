package org.firstinspires.ftc.teamcode.NATS;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Tip detection with the HuskyLens on the turret (AprilTag recognition mode).
 *
 * The tips are the hive goal cells (CELL 1 / CELL 2). Each cell has 4 AprilTags (see RobotConfig). Call
 * {@link #update(boolean, boolean)} every loop with the goal you are shooting at and it gives you:
 *   - {@link #hasTag()}        : a tag of that cell is in view right now (or within TIP_STALE_MS)
 *   - {@link #getAimErrorRad()}: how far the tag is left/right of the camera centre (for the turret fine aim)
 *   - {@link #getLevel()}      : UP (tag >= 40 in high), DOWN (< 30 in) or UNKNOWN (30-40 in / not seen)
 *   - {@link #needsSecondCycle()} : what the auto uses to decide on a second cycle
 *
 * The height is estimated from where the tag sits in the picture and how big it looks (pinhole model),
 * so it needs the camera numbers in RobotConfig (tag size, camera height, tilt, field of view) to be right.
 * The HuskyLens does not measure height directly: expect it to be good enough for UP vs DOWN (10 in apart),
 * not for exact inches.
 */
public class TipDetector {

    public enum Level { UP, DOWN, UNKNOWN }

    /** Read the camera at most this often (an I2C read takes a few ms). */
    private static final long READ_PERIOD_MS = 40;

    private HuskyLens lens;
    private boolean ok = false;

    private long lastReadMs = 0;
    private long lastSeenMs = 0;
    private int tagId = -1;
    private double aimErrorRad = 0;
    private double heightIn = Double.NaN;
    private double distanceIn = Double.NaN;
    private Level level = Level.UNKNOWN;

    // "Tipped" tracking: the highest height seen since resetTip(), and whether the tag has dropped
    // far enough below it. This is what the auto uses - a tip that falls reads as a lower tag.
    private double maxHeightIn = Double.NaN;
    private boolean tipped = false;

    public void init(HardwareMap hw) {
        try {
            lens = hw.get(HuskyLens.class, RobotConfig.HUSKYLENS);
            ok = lens.knock();
            lens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        } catch (Exception e) {
            ok = false; // no HuskyLens: everything reports "nothing seen", the robot still runs
        }
    }

    public boolean isConnected() { return ok; }

    /** Call every loop. red / cell1 pick which cell's 4 tags are looked for. */
    public void update(boolean red, boolean cell1) {
        if (lens == null || !ok) return;
        long now = System.currentTimeMillis();
        if (now - lastReadMs < READ_PERIOD_MS) {
            expireIfStale(now);
            return;
        }
        lastReadMs = now;

        int[] ids = idsFor(red, cell1);
        HuskyLens.Block best = null;
        HuskyLens.Block[] blocks = lens.blocks();
        if (blocks != null) {
            for (HuskyLens.Block b : blocks) {
                if (!contains(ids, b.id)) continue;
                if (best == null || b.height > best.height) best = b; // the biggest = the closest tag
            }
        }

        if (best != null && best.height > 0) {
            lastSeenMs = now;
            tagId = best.id;
            double f = focalPx();
            aimErrorRad = RobotConfig.CAM_X_SIGN * Math.atan((best.x - RobotConfig.CAM_IMAGE_W / 2.0) / f);

            // Pinhole model: z = distance straight out of the lens, u = how far above the lens axis the tag is.
            double z = f * RobotConfig.TAG_SIZE_IN / best.height;
            double u = z * (RobotConfig.CAM_IMAGE_H / 2.0 - best.y) / f;
            double pitch = Math.toRadians(RobotConfig.CAM_PITCH_DEG);
            double h = RobotConfig.CAM_HEIGHT_IN + z * Math.sin(pitch) + u * Math.cos(pitch);
            distanceIn = z * Math.cos(pitch) - u * Math.sin(pitch); // flat distance to the tag

            heightIn = Double.isNaN(heightIn)
                    ? h
                    : RobotConfig.TIP_HEIGHT_FILTER * heightIn + (1 - RobotConfig.TIP_HEIGHT_FILTER) * h;
            level = classify(heightIn);

            // A tip is "used" when its tag sits lower than it did: remember the highest reading and
            // flag the drop. Only runs while a tag is actually in view, so it is checked whenever
            // the camera can see one.
            if (Double.isNaN(maxHeightIn) || heightIn > maxHeightIn) {
                maxHeightIn = heightIn;
            } else if (!tipped && maxHeightIn - heightIn >= RobotConfig.TIP_DROP_IN) {
                tipped = true;
            }
        } else {
            expireIfStale(now);
        }
    }

    private void expireIfStale(long now) {
        if (lastSeenMs != 0 && now - lastSeenMs > RobotConfig.TIP_STALE_MS) {
            tagId = -1;
            level = Level.UNKNOWN;
            heightIn = Double.NaN;
            lastSeenMs = 0;
        }
    }

    private static Level classify(double h) {
        if (h >= RobotConfig.TIP_UP_MIN_IN) return Level.UP;
        if (h < RobotConfig.TIP_DOWN_MAX_IN) return Level.DOWN;
        return Level.UNKNOWN; // between 30 and 40 in: not sure either way
    }

    private static double focalPx() {
        return (RobotConfig.CAM_IMAGE_W / 2.0) / Math.tan(Math.toRadians(RobotConfig.CAM_HFOV_DEG / 2.0));
    }

    private static boolean contains(int[] ids, int id) {
        for (int i : ids) if (i == id) return true;
        return false;
    }

    public static int[] idsFor(boolean red, boolean cell1) {
        if (red) return cell1 ? RobotConfig.TAGS_RED_CELL1 : RobotConfig.TAGS_RED_CELL2;
        return cell1 ? RobotConfig.TAGS_BLUE_CELL1 : RobotConfig.TAGS_BLUE_CELL2;
    }

    // ---------------- results ----------------

    public boolean hasTag() { return tagId >= 0; }
    public int getTagId() { return tagId; }

    /** Arm the tip check: forget the heights seen so far. Call it when you start watching a hive. */
    public void resetTip() {
        maxHeightIn = Double.NaN;
        tipped = false;
    }

    /**
     * True once the tag has dropped TIP_DROP_IN or more below the highest height seen since
     * {@link #resetTip()}. Sticky: it stays true until the next reset, even if the tag leaves view.
     */
    public boolean hasTipped() { return tipped; }

    /** How far the tag has dropped below its highest reading since resetTip(), in inches. */
    public double getDropIn() {
        if (Double.isNaN(maxHeightIn) || Double.isNaN(heightIn)) return 0;
        return Math.max(0, maxHeightIn - heightIn);
    }
    /** Radians the tag is to the side of the camera centre, already signed for the turret (+ = turn CCW). */
    public double getAimErrorRad() { return aimErrorRad; }
    public double getHeightIn() { return heightIn; }
    public double getDistanceIn() { return distanceIn; }
    public Level getLevel() { return level; }

    /**
     * True when the tip tells the auto to run another cycle. UP counts when SECOND_CYCLE_WHEN_UP is true,
     * DOWN when it is false. UNKNOWN (30-40 in, or nothing seen) never triggers a second cycle by itself:
     * the auto's SECOND_CYCLE_IF_UNKNOWN setting decides that case.
     */
    public boolean needsSecondCycle() {
        if (level == Level.UNKNOWN) return RobotConfig.SECOND_CYCLE_IF_UNKNOWN;
        return (level == Level.UP) == RobotConfig.SECOND_CYCLE_WHEN_UP;
    }

    public void addTelemetry(Telemetry t) {
        if (!ok) {
            t.addData("HuskyLens", "not connected (check the name \"%s\")", RobotConfig.HUSKYLENS);
            return;
        }
        t.addData("Tip tag / level", "%s / %s", hasTag() ? "ID " + tagId : "none", level);
        t.addData("Tip height / distance (in)", "%.1f / %.1f", heightIn, distanceIn);
        t.addData("Tip drop", "%.1f in -> %s", getDropIn(), tipped ? "TIPPED" : "not yet");
        t.addData("Second cycle?", needsSecondCycle());
    }
}
