package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.turret.HiveTracker;

/**
 * Hand-off from auto to TeleOp (the static-field approach from the Pedro docs' "Passing Pose from Autonomous to
 * TeleOp"). Static fields live as long as the Robot Controller app, so they survive the AUTO -> TELEOP switch
 * but also survive into unrelated later runs; {@link #hasFreshAutoState()} guards against using stale data.
 */
public final class MatchState {
    private MatchState() {}

    /** Older than this and TeleOp ignores it (AUTO 30 s + 8 s transition + time to INIT TeleOp, with margin). */
    public static double MAX_AGE_SEC = 120.0;

    public static Pose pose = null;
    public static Alliance alliance = Alliance.RED;
    public static HiveTracker.Cell raisedCell = null;
    public static double hiveCorrectionX = 0, hiveCorrectionY = 0;
    public static boolean hiveSeenByCamera = false;
    private static long savedAtNanos = 0;

    /** Call every loop in auto and once more in stop(), so the last known state is always saved. */
    public static void saveFromAuto(Pose robotPose, Alliance autoAlliance, HiveTracker tracker) {
        pose = robotPose;
        alliance = autoAlliance;
        raisedCell = tracker.raisedCell();
        double[] correction = tracker.correction();
        hiveCorrectionX = correction[0];
        hiveCorrectionY = correction[1];
        hiveSeenByCamera = tracker.hasSeenHive();
        savedAtNanos = System.nanoTime();
    }

    public static boolean hasFreshAutoState() {
        return pose != null && savedAtNanos != 0 && ageSeconds() <= MAX_AGE_SEC;
    }

    public static double ageSeconds() {
        return (System.nanoTime() - savedAtNanos) / 1e9;
    }

    /** Called by TeleOp after using it, so restarting TeleOp later doesn't reuse a stale pose. */
    public static void clear() {
        pose = null;
        raisedCell = null;
        savedAtNanos = 0;
    }
}
