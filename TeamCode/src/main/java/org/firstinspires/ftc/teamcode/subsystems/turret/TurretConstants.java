package org.firstinspires.ftc.teamcode.subsystems.turret;

/**
 * Every tunable number for the turret lives here. Units: inches, degrees, seconds unless noted.
 *
 * Frames:
 *  - Field: Pedro coordinates (inches, origin in a corner, heading CCW positive).
 *  - Robot frame: +X forward, +Y left (same as Pedro). Origin = center of the four mecanum wheels.
 *  - Turret angle: 0 = pointing straight forward, positive = CCW (turret turning to the robot's left).
 *  - Camera pitch: 0 = lens level, positive = tilted up.
 *
 * Geometry marked "CAD" comes from "full robot assembly.step" (2026-10-27). The robot is not built yet and the
 * CAD will change (the turret is being redesigned), so re-measure those values on the real robot. Heights are from
 * the tiles: CAD z = 0 is the bottom of the frame, about 0.31 in. above the floor.
 */
public final class TurretConstants {
    private TurretConstants() {}

    // ---------------- Hardware names ----------------
    public static String SERVO_A_NAME = "turretServoA";
    public static String SERVO_B_NAME = "turretServoB";
    /** Analog port with servo A's feedback wire. "" = not wired (falls back to the slew-rate model). */
    public static String FEEDBACK_NAME = "turretFeedback";
    public static String CAMERA_NAME = "TurretCam";

    // ---------------- Servo <-> turret angle mapping ----------------
    // Two Axon Minis, each with a 64T goBILDA gear, drive the 176T (MOD 0.8) goBILDA turntable. Both sit under
    // the ring with their shafts pointing up, so the same servo command turns both pinions the same way.
    // They MUST agree or they will fight and overheat. Calibrate each one on its own (other servo's gear
    // disengaged): find the servo position that points the turret straight forward.
    /** Turntable teeth / pinion teeth. */
    public static double TURRET_GEAR_RATIO = 176.0 / 64.0;
    /** Servo A position when the turret points straight forward. */
    public static double SERVO_A_ZERO = 0.5;
    /** Servo B position when the turret points straight forward. */
    public static double SERVO_B_ZERO = 0.5;
    /** +1 if increasing servo position turns the turret left (CCW seen from above), -1 if right. */
    public static double SERVO_A_DIRECTION = 1.0;
    public static double SERVO_B_DIRECTION = 1.0;

    // ---------------- Feedback (servo A's analog wire) ----------------
    // Calibrate with TurretTrackingTest: hold the turret at two angles, note "servo A cmd" and "feedback V" for
    // each, and extrapolate the voltage at positions 0 and 1. Equal values = feedback ignored.
    public static double FEEDBACK_VOLTS_AT_ZERO = 0.0;
    public static double FEEDBACK_VOLTS_AT_ONE = 3.3;

    // ---------------- Travel limits ----------------
    // With the 2.75:1 gear the full 355 deg servo sweep is only ~129 deg of turret, i.e. about +/-64 deg around
    // forward if SERVO_A_ZERO is 0.5. Shrink these once the wire routing is known.
    public static double MIN_ANGLE_DEG = -60.0;
    public static double MAX_ANGLE_DEG = 60.0;

    // ---------------- Servo motion model (used only without feedback) ----------------
    /** Measured turret slew rate under load, with the flywheels mounted. Time a large move to find it. */
    public static double SERVO_MAX_DEG_PER_SEC = 180.0;
    /** How far ahead to predict robot motion when aiming, to cancel servo + loop lag. */
    public static double AIM_LOOKAHEAD_SEC = 0.08;

    // ---------------- Mounting geometry ----------------
    /** Turret pivot (turntable center) in the robot frame. CAD: 1.17 in. behind the wheel center. */
    public static double TURRET_OFFSET_X = -1.17;
    public static double TURRET_OFFSET_Y = 0.0;
    /** Camera yaw relative to the turret's pointing direction (positive = camera aimed left of the shooter). */
    public static double CAMERA_YAW_DEG = 0.0;

    // ---------------- Camera tilt (Axon Mini on the turret, camera on a 3D-printed arm) ----------------
    public static String TILT_SERVO_NAME = "cameraTilt";
    /** Analog port with the tilt servo's feedback wire. "" = not wired. */
    public static String TILT_FEEDBACK_NAME = "cameraTiltFeedback";
    /** Tilt servo position when the lens points level. 0.29 puts the CAD pose (75.5 deg up) near mid-travel. */
    public static double TILT_SERVO_LEVEL = 0.29;
    /** +1 if increasing servo position tilts the camera up, -1 if down. The servo drives the arm 1:1. */
    public static double TILT_DIRECTION = 1.0;
    public static double TILT_FEEDBACK_VOLTS_AT_ZERO = 0.0;
    public static double TILT_FEEDBACK_VOLTS_AT_ONE = 3.3;
    /** Allowed pitch range. 640x480 has only ~47 deg vertical field of view, so the camera follows the CELL. */
    public static double MIN_PITCH_DEG = 15.0;
    public static double MAX_PITCH_DEG = 80.0;
    /** Pitch while the turret is not tracking (CAD pose). */
    public static double DEFAULT_PITCH_DEG = 75.5;
    /** Tilt slew rate (used only without feedback). */
    public static double TILT_MAX_DEG_PER_SEC = 400.0;
    /**
     * Tilt axis in the turret frame. CAD: the servo shaft is horizontal (left-right), 0.64 in. ahead of the turret
     * pivot and 10.37 in. above the floor, in line with the lens 3.33 in. left of the pivot.
     */
    public static double TILT_AXIS_FORWARD = 0.64;
    public static double TILT_AXIS_HEIGHT = 10.37;
    public static double CAMERA_LEFT = 3.33;
    /**
     * Lens position relative to the tilt axis, measured in the camera's own frame: along the lens direction and
     * along the image "up" direction. CAD: the lens sits on the optical axis, 2.43 in. out from the tilt axis.
     * Update these when the 3D-printed arm is designed.
     */
    public static double LENS_ALONG_AXIS = 2.43;
    public static double LENS_ABOVE_AXIS = 0.0;

    // ---------------- Vision ----------------
    // Must be a calibrated size for the goBILDA global shutter camera (3122-0004-0001, calibration in
    // res/xml/teamwebcamcalibrations.xml): 1280x800, 1280x720, 800x600, 640x480 or 480x320.
    // Uncalibrated sizes give wrong distances.
    public static int CAMERA_WIDTH = 640;
    public static int CAMERA_HEIGHT_PX = 480;
    /** AprilTag decimation: higher = faster but shorter detection range. */
    public static float APRILTAG_DECIMATION = 2.0f;
    /**
     * Manual exposure (ms) so tags still detect while the robot moves. Find it with Camera Bench Test (dpad
     * up/down). 0 = leave the camera on auto exposure.
     */
    public static int EXPOSURE_MS = 0;
    /** Manual gain to go with EXPOSURE_MS (Camera Bench Test dpad left/right). -1 = leave it as it is. */
    public static int GAIN = -1;
    /** Used if the frame timestamp looks wrong. */
    public static double DEFAULT_CAMERA_LATENCY_SEC = 0.05;
    /** Ignore detections farther than this (horizontal); pose noise grows quickly with range. */
    public static double MAX_TRUSTED_RANGE = 120.0;

    // ---------------- Field geometry (BIOBUZZ manual section 9.6, +/- 1 in.) ----------------
    // The HIVE frame is anchored at field center. Each alliance's HIVE is a seesaw with a CELL at each end;
    // exactly one CELL is raised. AprilTag cluster origins sit at the center of each CELL opening.
    // The positions below are estimates from the manual drawings; the camera correction absorbs the error.
    /** Field center in Pedro coordinates. */
    public static double FIELD_CENTER_X = 72.0;
    public static double FIELD_CENTER_Y = 72.0;
    /**
     * Pedro-frame direction (deg) pointing from the audience toward the far wall (the "SCORING" CELL side).
     * 90 assumes the Pedro field picture is drawn with the audience at the bottom and red on the left.
     * VERIFY on the real field: drive toward the far wall and check that Pedro's heading reads this value.
     */
    public static double FAR_WALL_DIRECTION_DEG = 90.0;
    /** HIVE center-to-center is 25.5 in., so each HIVE is 12.75 in. toward its own alliance wall from center. */
    public static double HIVE_SIDE_OFFSET = 12.75;
    /** Horizontal distance from field center (toward its own end) of a RAISED CELL's opening center. */
    public static double RAISED_CELL_REACH = 8.8;
    /** Horizontal distance from field center (toward its own end) of a LOWERED CELL's opening center. */
    public static double LOWERED_CELL_REACH = 17.9;
    /** Raised opening spans 53.5-65.6 in. above the tiles. */
    public static double RAISED_CELL_HEIGHT = 59.5;
    public static double LOWERED_CELL_HEIGHT = 44.0;

    // ---------------- Raised-side detection ----------------
    /** A sighted CELL above this height is raised, below it is lowered (then the other CELL is the raised one). */
    public static double RAISED_HEIGHT_THRESHOLD = 52.0;
    /** Ignore sightings within this band around the threshold (ambiguous, probably mid-tip). */
    public static double RAISED_HEIGHT_DEADBAND = 3.0;
    /** Consecutive agreeing frames needed before we believe the HIVE tipped. */
    public static int FRAMES_TO_CONFIRM_TIP = 3;

    // ---------------- Camera correction filter ----------------
    /** Blend factor per camera frame for a fully visible cluster (0..1). Higher = trusts camera more, noisier. */
    public static double FILTER_GAIN = 0.3;
    /** A sighting whose implied correction differs from the current one by more than this is an outlier... */
    public static double OUTLIER_DISTANCE = 8.0;
    /** ...unless this many outliers in a row agree (e.g. odometry got bumped), then snap to it. */
    public static int OUTLIER_FRAMES_TO_RESET = 4;

    // ---------------- Aiming ----------------
    /** Turret is "on target" when the estimated angle is within this of the desired angle. */
    public static double ON_TARGET_TOLERANCE_DEG = 2.0;
}
