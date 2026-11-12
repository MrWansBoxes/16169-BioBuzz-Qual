package org.firstinspires.ftc.teamcode.subsystems.pollen;

import org.firstinspires.ftc.vision.opencv.ColorRange;

/**
 * Tunables for finding POLLEN with the front camera. Units: inches, degrees, seconds, pixels unless noted.
 * Robot frame: +X forward, +Y left, origin = center of the four mecanum wheels (same as the turret).
 *
 * Other teams put yellow on their robots to fool cameras, so yellow is only the first of five stages
 * (see {@link PollenTracker}). Tune them in order with PollenCameraTest; the "Robot Tuning Guide" doc says how.
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
    /** Used if a frame's capture timestamp looks wrong. */
    public static double CAMERA_LATENCY_SEC = 0.05;

    // ---------------- Mounting (CAD 2026-11-12: centered on the front, looking forward) ----------------
    // Heights are from the tiles. CAD z = 0 is the bottom of the frame, about 0.31 in. above the floor.
    public static double CAMERA_FORWARD = 7.98;
    public static double CAMERA_LEFT = 0.0;
    public static double CAMERA_HEIGHT = 5.68;
    /** Negative = tilted down. */
    public static double CAMERA_PITCH_DEG = -17.0;

    // ---------------- The ball ----------------
    /** CAD: 73.7 mm hollow ball. Its center is assumed to sit this high / 2 above the tiles. Measure a real one. */
    public static double POLLEN_DIAMETER = 2.90;
    /** Hole radius / ball radius. CAD: 6.4 mm holes on a 36.8 mm radius ball. */
    public static double HOLE_RADIUS_RATIO = 0.174;

    // ---------------- Stage 1: yellow candidates below the floor line ----------------
    /** POLLEN is bright yellow. Only has to catch every real ball (decoys are rejected later). */
    public static ColorRange COLOR = ColorRange.YELLOW;
    /**
     * Only the image below the "floor line" is searched: the row where the top of a ball MAX_RANGE away appears.
     * Anything above it is too high to be a ball on the floor. This many extra rows are searched above the line.
     */
    public static int ROI_MARGIN_PX = 20;
    public static int MIN_AREA_PX = 150;
    public static int MAX_AREA_PX = 60000;
    /** 1 = perfect circle. Holes along the edge make the outline bumpy, so don't set this too high. */
    public static double MIN_CIRCULARITY = 0.5;
    /** Ignore anything farther than this from the robot center. */
    public static double MAX_RANGE = 72.0;

    // ---------------- Stage 2: ball-sized for its distance ----------------
    /** Measured / expected blob diameter must be within 1 +/- this. */
    public static double SIZE_TOLERANCE = 0.3;

    // ---------------- Stage 3: holes, up close ----------------
    /** Closer than this (from the robot center), a candidate must show holes. Farther, holes are too small. */
    public static double HOLE_CHECK_RANGE_IN = 36.0;
    public static int MIN_HOLES = 3;
    /** A spot is a hole if it is darker than this fraction of the ball's median brightness. */
    public static double HOLE_DARKNESS = 0.75;
    /** Holes are searched within this fraction of the ball's radius (the rim is shaded and foreshortened). */
    public static double HOLE_SEARCH_FRACTION = 0.8;

    // ---------------- Stage 4: still on the field ----------------
    /** A sighting must stay within STILL_RADIUS_IN of where it was first seen for STILL_FRAMES camera frames. */
    public static int STILL_FRAMES = 3;
    public static double STILL_RADIUS_IN = 1.0;

    // ---------------- Stage 5: inside the field ----------------
    /** Field size (Pedro coordinates, 0..FIELD_SIZE on both axes). */
    public static double FIELD_SIZE = 144.0;
    /** A ball's center must be at least this far inside the walls (about its radius). */
    public static double FIELD_MARGIN_IN = 1.5;
    /** Ignore sightings within this distance of each HIVE center (positions from TurretConstants). 0 = off. */
    public static double HIVE_KEEPOUT_RADIUS = 0.0;

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
