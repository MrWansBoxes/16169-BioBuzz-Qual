package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.*;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.AxonServo;

import java.util.function.Supplier;

/**
 * Turret: two Axon Minis with 64T pinions on the 176T turntable, plus the camera tilt servo ({@link CameraTilt}).
 *
 * Each loop, {@link #update()}:
 *  1. reads where the turret really is from servo A's feedback wire (or, if that is not wired, advances a
 *     slew-rate model of it),
 *  2. records (time, robot pose, turret angle, camera pitch) so camera frames can be matched to the moment they
 *     were taken,
 *  3. lets {@link HiveTracker} fold in any new AprilTag sighting,
 *  4. aims at the tracker's aim point from the robot pose predicted {@link TurretConstants#AIM_LOOKAHEAD_SEC}
 *     ahead, which cancels most of the lag when the robot drives or spins,
 *  5. picks the reachable equivalent of that angle inside the travel limits and writes both servos,
 *  6. tilts the camera so the CELL it is aiming at stays in view.
 *
 * This object is also the Ivy requirement for turret commands (see {@link TurretCommands}).
 */
public class Turret {
    public enum Mode { TRACK, FIXED }

    private final AxonServo servoA;
    private final AxonServo servoB;
    private final CameraTilt tilt;
    private final Supplier<Pose> poseSupplier;
    private final HiveTracker tracker;
    private final PoseHistory history = new PoseHistory(64);

    private Mode mode = Mode.FIXED;
    private double fixedRad = 0;
    private double trimRad = 0;

    private double desiredRad = 0;
    private double commandedRad = 0;
    private double estimatedRad = 0;
    private double measuredRad = 0;
    private boolean reachable = true;
    private double targetDistance = Double.NaN;
    private long lastNanos = 0;

    public Turret(HardwareMap hardwareMap, Supplier<Pose> poseSupplier, HiveTracker tracker) {
        this.servoA = new AxonServo(hardwareMap, SERVO_A_NAME, FEEDBACK_NAME);
        this.servoB = new AxonServo(hardwareMap, SERVO_B_NAME, "");
        this.tilt = new CameraTilt(hardwareMap);
        this.poseSupplier = poseSupplier;
        this.tracker = tracker;
    }

    /** Call exactly once per loop, after the follower has updated. */
    public void update() {
        long now = System.nanoTime();
        double dt = lastNanos == 0 ? 0 : (now - lastNanos) / 1e9;
        lastNanos = now;

        double maxStep = Math.toRadians(SERVO_MAX_DEG_PER_SEC) * dt;
        estimatedRad += Range.clip(commandedRad - estimatedRad, -maxStep, maxStep);
        double feedbackPosition = servoA.measuredPosition(FEEDBACK_VOLTS_AT_ZERO, FEEDBACK_VOLTS_AT_ONE);
        measuredRad = Double.isNaN(feedbackPosition)
                ? estimatedRad
                : Math.toRadians((feedbackPosition - SERVO_A_ZERO) * degreesPerUnit(SERVO_A_DIRECTION));
        tilt.update(dt);

        Pose pose = poseSupplier.get();
        history.add(now, pose, measuredRad, tilt.pitchRad());
        tracker.update(history);

        desiredRad = mode == Mode.TRACK ? trackingAngle(pose) : fixedRad;
        commandedRad = reachableAngle(desiredRad + trimRad);
        writeServos(commandedRad);

        if (mode == Mode.TRACK) {
            tilt.aimAt(targetDistance - TILT_AXIS_FORWARD, tracker.aimHeight());
        } else {
            tilt.setPitch(Math.toRadians(DEFAULT_PITCH_DEG));
        }
    }

    private double trackingAngle(Pose pose) {
        double[] v = history.velocity(0.1);
        double t = AIM_LOOKAHEAD_SEC;
        double heading = pose.heading() + v[2] * t;
        double x = pose.x() + v[0] * t;
        double y = pose.y() + v[1] * t;

        double pivotX = x + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double pivotY = y + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);
        double[] aim = tracker.aimPoint();
        double dx = aim[0] - pivotX;
        double dy = aim[1] - pivotY;
        targetDistance = Math.hypot(dx, dy);
        return Angle.normalizeSigned(Math.atan2(dy, dx) - heading);
    }

    /** Equivalent angle (+/- 360) inside the travel limits closest to where we are, else the nearer limit. */
    private double reachableAngle(double angle) {
        double min = Math.toRadians(MIN_ANGLE_DEG);
        double max = Math.toRadians(MAX_ANGLE_DEG);
        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double candidate = angle + k * 2 * Math.PI;
            if (candidate >= min && candidate <= max
                    && (Double.isNaN(best) || Math.abs(candidate - commandedRad) < Math.abs(best - commandedRad))) {
                best = candidate;
            }
        }
        reachable = !Double.isNaN(best);
        if (reachable) return best;
        double normalized = Angle.normalizeSigned(angle);
        return Math.abs(Angle.normalizeSigned(normalized - min)) < Math.abs(Angle.normalizeSigned(normalized - max))
                ? min : max;
    }

    private void writeServos(double angleRad) {
        double deg = Math.toDegrees(angleRad);
        servoA.setPosition(SERVO_A_ZERO + deg / degreesPerUnit(SERVO_A_DIRECTION));
        servoB.setPosition(SERVO_B_ZERO + deg / degreesPerUnit(SERVO_B_DIRECTION));
    }

    /** Turret degrees per unit of servo position. */
    private static double degreesPerUnit(double direction) {
        return direction * AxonServo.RANGE_DEG / TURRET_GEAR_RATIO;
    }

    public void track() {
        mode = Mode.TRACK;
    }

    /** Hold a fixed angle relative to the robot (0 = forward). */
    public void holdAngle(double degrees) {
        mode = Mode.FIXED;
        fixedRad = Math.toRadians(degrees);
    }

    /** Driver fine-tune added on top of whatever the turret is doing. */
    public void setTrimDegrees(double degrees) {
        trimRad = Math.toRadians(degrees);
    }

    public double trimDegrees() {
        return Math.toDegrees(trimRad);
    }

    /** Ready to shoot: tracking, target inside travel, and the turret has finished moving. */
    public boolean isOnTarget() {
        return mode == Mode.TRACK && reachable
                && Math.abs(commandedRad - measuredRad) < Math.toRadians(ON_TARGET_TOLERANCE_DEG);
    }

    public Mode mode() {
        return mode;
    }

    /** Horizontal distance from the turret pivot to the raised CELL opening, for flywheel speed lookup. */
    public double targetDistance() {
        return targetDistance;
    }

    public double targetHeight() {
        return tracker.aimHeight();
    }

    public HiveTracker tracker() {
        return tracker;
    }

    public CameraTilt tilt() {
        return tilt;
    }

    public void addTelemetry(Telemetry telemetry) {
        double[] aim = tracker.aimPoint();
        double[] corr = tracker.correction();
        telemetry.addData("Turret mode", mode);
        String angleLabel = servoA.hasFeedback() ? "measured" : "est";
        telemetry.addData("Turret desired / cmd / " + angleLabel + " (deg)", "%.1f / %.1f / %.1f",
                Math.toDegrees(desiredRad), Math.toDegrees(commandedRad), Math.toDegrees(measuredRad));
        telemetry.addData("Turret servo A cmd / feedback V", "%.3f / %.3f",
                SERVO_A_ZERO + Math.toDegrees(commandedRad) / degreesPerUnit(SERVO_A_DIRECTION),
                servoA.feedbackVolts());
        telemetry.addData("Camera pitch cmd / actual (deg), feedback V", "%.1f / %.1f, %.3f",
                Math.toDegrees(tilt.commandedPitchRad()), Math.toDegrees(tilt.pitchRad()), tilt.feedbackVolts());
        telemetry.addData("Turret reachable / on target", "%b / %b", reachable, isOnTarget());
        telemetry.addData("Trim (deg)", "%.1f", trimDegrees());
        telemetry.addData("Raised CELL", "%s (camera confirmed: %b)", tracker.raisedCell(), tracker.hasSeenHive());
        telemetry.addData("Aim point", "(%.1f, %.1f)  dist %.1f", aim[0], aim[1], targetDistance);
        telemetry.addData("Camera correction", "(%.1f, %.1f)", corr[0], corr[1]);
        telemetry.addData("Last sighting", "%s  h=%.1f in  %d%% tags  %.1fs ago",
                tracker.lastSeenCell(), tracker.lastSeenHeight(), tracker.lastPercentFound(), tracker.secondsSinceSeen());
        telemetry.addData("Camera", tracker.cameraState());
    }
}
