package org.firstinspires.ftc.teamcode.subsystems.pollen;

import org.firstinspires.ftc.vision.opencv.ColorRange;

/**
 * Tunables for finding POLLEN with the front camera. Units: inches, degrees, seconds, pixels unless noted.
 * Robot frame: +X forward, +Y left, origin = center of the four mecanum wheels (same as the turret).
 */
public final class PollenConstants {
    private PollenConstants() {}

    public static String CAMERA_NAME = "PollenCam";

    // ---------------- Camera (goBILDA global shutter 3122-0004-0001) ----------------
    // Must be one of the calibrated sizes in res/xml/teamwebcamcalibrations.xml, and the intrinsics below must
    // be copied from that size's entry (they are used to turn pixels into floor positions).
    public static int CAMERA_WIDTH = 640;
    public static int CAMERA_HEIGHT_PX = 480;
    public static double FX = 545.584;
    public static double FY = 545.584;
    public static double CX = 350.588;
    public static double CY = 223.150;

    // ---------------- Mounting (CAD 2026-10-27: centered on the front, looking forward) ----------------
    public static double CAMERA_FORWARD = 7.98;
    public static double CAMERA_LEFT = 0.0;
    public static double CAMERA_HEIGHT = 5.37;
    /** Negative = tilted down. */
    public static double CAMERA_PITCH_DEG = -17.0;

    // ---------------- Detection ----------------
    /** POLLEN is bright yellow. The SDK's YELLOW range (YCrCb) is a good start; tune with PollenCameraTest. */
    public static ColorRange COLOR = ColorRange.YELLOW;
    /** Ball diameter. Its center is assumed to sit this high / 2 above the tiles. Measure a real POLLEN. */
    public static double POLLEN_DIAMETER = 5.0;
    public static int MIN_AREA_PX = 150;
    public static int MAX_AREA_PX = 60000;
    /** 1 = perfect circle. Lower accepts partly hidden balls but also more junk. */
    public static double MIN_CIRCULARITY = 0.5;
    /** Ignore anything farther than this from the robot center. */
    public static double MAX_RANGE = 72.0;
    /** {@code hasTarget()} turns false if nothing has been seen for this long. */
    public static double LOST_AFTER_SEC = 0.3;

    // ---------------- TeleOp assist (turn toward the nearest POLLEN) ----------------
    /** Turn power per radian of bearing error. */
    public static double ASSIST_KP = 1.2;
    public static double ASSIST_MAX_TURN = 0.6;

    // ---------------- Auto pickup ----------------
    /** Where the ball should be (forward of the robot center) when the pickup path ends: inside the roller bed. */
    public static double PICKUP_BALL_FORWARD = 6.0;
    public static double PICKUP_TIMEOUT_MS = 3000;
}
