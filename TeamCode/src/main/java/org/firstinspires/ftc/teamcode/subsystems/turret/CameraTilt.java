package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.AxonServo;

/**
 * The turret camera's tilt servo. The CELL tags face down from ~44-60 in. up, and at 640x480 the camera only
 * sees ~47 deg vertically, so one fixed pitch can't cover both close and far shots. The turret points the lens
 * at the CELL it is aiming at ({@link #aimAt}); {@link HiveTracker} uses the pitch at the moment each frame was
 * captured to turn a detection into a field position.
 *
 * Owned and updated by {@link Turret}.
 */
public class CameraTilt {
    private final AxonServo servo;
    private double commandedRad = Math.toRadians(DEFAULT_PITCH_DEG);
    private double estimatedRad = commandedRad;

    public CameraTilt(HardwareMap hardwareMap) {
        servo = new AxonServo(hardwareMap, TILT_SERVO_NAME, TILT_FEEDBACK_NAME);
        servo.setPosition(servoPosition(commandedRad));
    }

    /** Point the lens at a spot this far away horizontally (from the tilt axis) and this high above the floor. */
    public void aimAt(double horizontalDistance, double height) {
        if (Double.isNaN(horizontalDistance)) {
            setPitch(Math.toRadians(DEFAULT_PITCH_DEG));
            return;
        }
        // The lens sits on the optical axis through the tilt axis (LENS_ABOVE_AXIS ~ 0), so pointing that axis
        // straight at the target centers it in the image.
        setPitch(Math.atan2(height - TILT_AXIS_HEIGHT, Math.max(horizontalDistance, 1.0)));
    }

    public void setPitch(double pitchRad) {
        commandedRad = Range.clip(pitchRad, Math.toRadians(MIN_PITCH_DEG), Math.toRadians(MAX_PITCH_DEG));
        servo.setPosition(servoPosition(commandedRad));
    }

    /** Advance the slew model (only matters without feedback). Call once per loop. */
    void update(double dt) {
        double maxStep = Math.toRadians(TILT_MAX_DEG_PER_SEC) * dt;
        estimatedRad += Range.clip(commandedRad - estimatedRad, -maxStep, maxStep);
    }

    /** Best guess of the real pitch: the feedback wire if wired and calibrated, else the slew model. */
    public double pitchRad() {
        double position = servo.measuredPosition(TILT_FEEDBACK_VOLTS_AT_ZERO, TILT_FEEDBACK_VOLTS_AT_ONE);
        if (Double.isNaN(position)) return estimatedRad;
        return Math.toRadians((position - TILT_SERVO_LEVEL) * TILT_DIRECTION * AxonServo.RANGE_DEG);
    }

    public double commandedPitchRad() {
        return commandedRad;
    }

    public double feedbackVolts() {
        return servo.feedbackVolts();
    }

    private static double servoPosition(double pitchRad) {
        return TILT_SERVO_LEVEL + Math.toDegrees(pitchRad) / (TILT_DIRECTION * AxonServo.RANGE_DEG);
    }

    /**
     * Lens position in the turret frame at this pitch: {forward of the turret pivot, left of it, height above
     * the floor}. The lens swings around the tilt axis, so it moves when the camera tilts.
     */
    public static double[] lensPosition(double pitchRad) {
        double cos = Math.cos(pitchRad), sin = Math.sin(pitchRad);
        return new double[]{
                TILT_AXIS_FORWARD + LENS_ALONG_AXIS * cos - LENS_ABOVE_AXIS * sin,
                CAMERA_LEFT,
                TILT_AXIS_HEIGHT + LENS_ALONG_AXIS * sin + LENS_ABOVE_AXIS * cos
        };
    }
}
