package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.TURRET_OFFSET_Y;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.turret.HiveTracker;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Turret camera bench test: needs ONLY the turret webcam (configured as "TurretCam"); no motors or servos.
 * Runs the real {@link HiveTracker} code as if the robot sat still at {@link #BENCH_POSE} with the turret
 * pointing straight ahead, and shows every number so it can be checked with a tape measure.
 *
 * What to test (print the BIOBUZZ AprilTag cluster sheet at 100% scale from the Playing Field Resources page;
 * on the field the clusters face DOWN from the bottom of each CELL):
 *  1. Detection: "Detections" lists the cluster by name (e.g. RED AUDIENCE) and % of its 4 tags found.
 *  NOTE: a cluster's reported position is its ORIGIN, which the SDK puts at the center of the CELL opening, a
 *  few inches off the sticker (offset 7.19 in. along the sheet and 5.62 in. out of it). So on a bench compare
 *  CHANGES, not absolute numbers:
 *  2. Lens calibration: move the sheet exactly 12 in. farther from the camera; "raw range" should grow by
 *     ~12 in. at several distances. If not, the camera needs calibrating (the SDK only has a built-in one for
 *     some cameras).
 *  3. Mount geometry: set BENCH_PITCH_DEG below to the bench camera's tilt (heights are reported as if the
 *     camera were on the turret, per the TILT_* / LENS_* constants in TurretConstants). Raise the sheet 6 in.:
 *     "height" should rise ~6 in. and "horizontal" stay the same. If horizontal changes too, the pitch is wrong.
 *  4. Raised/lowered: on the field a raised opening is ~59.5 in. up, lowered ~44 in. Check the RAISED/LOWERED
 *     line on the real field (or a mock CELL with the sheet on its underside).
 *  5. Exposure: wave the sheet; raise/lower exposure until tags still detect while moving (less blur).
 *
 * Init or run: X = blue, B = red (alliance used for raised-CELL logic).
 * Run: dpad up/down = exposure +/- 1 ms, dpad right/left = gain +/- 10, A = cycle decimation 1-4.
 * The Driver Station camera stream (3-dot menu > Camera Stream, during INIT) shows the image with tag outlines.
 */
@TeleOp(name = "Camera Bench Test", group = "Test")
public class CameraBenchTest extends OpMode {
    /** Where to pretend the bench camera is (Pedro coordinates). Only matters for the aim point / correction. */
    public static Pose BENCH_POSE = new Pose(72, 24, Math.toRadians(90));
    /** Camera tilt on the bench (positive = up). On the robot the tilt servo sets this. */
    public static double BENCH_PITCH_DEG = 45.0;

    private static final float[] DECIMATIONS = {1, 2, 3, 4};

    private HiveTracker tracker;
    private int decimationIndex = 1;
    private boolean cameraControlsRead = false;
    private long exposureMs = 0, minExposureMs = 0, maxExposureMs = 0;
    private int gain = 0, minGain = 0, maxGain = 0;
    private boolean lastA, lastUp, lastDown, lastLeft, lastRight;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(100);
        tracker = new HiveTracker(hardwareMap, Alliance.RED, HiveTracker.DEFAULT_LIVE_VIEW);
        tracker.setDecimation(DECIMATIONS[decimationIndex]);
    }

    @Override
    public void init_loop() {
        runBench(false);
    }

    @Override
    public void loop() {
        runBench(true);
    }

    private void runBench(boolean running) {
        if (gamepad1.x && tracker.alliance() != Alliance.BLUE) tracker.setAlliance(Alliance.BLUE);
        if (gamepad1.b && tracker.alliance() != Alliance.RED) tracker.setAlliance(Alliance.RED);
        if (running) handleCameraControls();

        tracker.updateStationary(BENCH_POSE, 0, BENCH_PITCH_DEG);

        telemetry.addData("Camera", "%s  %.0f fps  decimation %.0f",
                tracker.cameraState(), tracker.portal().getFps(), DECIMATIONS[decimationIndex]);
        if (cameraControlsRead) {
            telemetry.addData("Exposure / gain", "%d ms (%d-%d) / %d (%d-%d)",
                    exposureMs, minExposureMs, maxExposureMs, gain, minGain, maxGain);
        }
        telemetry.addData("Alliance (X blue / B red)", tracker.alliance());

        showDetections(tracker.latestDetections());
        showTrackerResult();
        if (!running) telemetry.addLine("\nPress START for exposure / gain / decimation controls.");
    }

    private void showDetections(List<AprilTagDetection> detections) {
        telemetry.addLine(String.format("\n==== Detections: %d", detections.size()));
        for (AprilTagDetection d : detections) {
            String label;
            String extra = "";
            if (d instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection c = (AprilTagClusterDetection) d;
                label = c.metadata.name;
                extra = String.format("  %d%% of tags", c.percentClusterFound);
            } else if (d instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection s = (AprilTagSingleDetection) d;
                label = s.metadata != null ? "tag " + s.id + " (" + s.metadata.name + ")" : "unknown tag " + s.id;
            } else {
                label = d.getClass().getSimpleName();
            }
            telemetry.addLine(String.format("-- %s%s", label, extra));
            telemetry.addLine(String.format("   raw range %.1f in  bearing %.1f°  elevation %.1f°",
                    d.ftcPose.range, Math.toDegrees(d.ftcPose.bearing), Math.toDegrees(d.ftcPose.elevation)));

            HiveTracker.Measurement m = HiveTracker.measure(d, Math.toRadians(BENCH_PITCH_DEG));
            telemetry.addLine(String.format("   turret frame: fwd %.1f  left %.1f  height %.1f in",
                    m.forward, m.left, m.height));
            telemetry.addLine(String.format("   horizontal %.1f in at %.1f° (turn turret this way to center it)",
                    m.range, Math.toDegrees(m.bearing)));

            if (d instanceof AprilTagClusterDetection) {
                Boolean raised = HiveTracker.isRaisedHeight(m.height);
                String state = raised == null ? "AMBIGUOUS (near threshold)" : raised ? "RAISED" : "LOWERED";
                boolean ours = tracker.alliance().ownsCluster(((AprilTagClusterDetection) d).metadata.name);
                telemetry.addLine(String.format("   -> %s, %s", state, ours ? "OUR alliance" : "other alliance"));
            }
        }
    }

    private void showTrackerResult() {
        double[] aim = tracker.aimPoint();
        double[] correction = tracker.correction();
        double heading = BENCH_POSE.heading();
        double pivotX = BENCH_POSE.x() + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double pivotY = BENCH_POSE.y() + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);
        double turretDeg = Math.toDegrees(Angle.normalizeSigned(Math.atan2(aim[1] - pivotY, aim[0] - pivotX) - heading));

        telemetry.addLine("\n==== Tracker (robot assumed still at BENCH_POSE)");
        telemetry.addData("Raised CELL", "%s  (camera confirmed: %b)", tracker.raisedCell(), tracker.hasSeenHive());
        telemetry.addData("Last sighting", "%s  h=%.1f in  %.1f s ago",
                tracker.lastSeenCell(), tracker.lastSeenHeight(), tracker.secondsSinceSeen());
        telemetry.addData("Aim point (field)", "(%.1f, %.1f)", aim[0], aim[1]);
        telemetry.addData("Correction", "(%.1f, %.1f)  (only meaningful with the bench at BENCH_POSE on a field)",
                correction[0], correction[1]);
        telemetry.addData("Turret would point", "%.1f° (0 = forward, + = left)", turretDeg);
    }

    private void handleCameraControls() {
        if (tracker.cameraState() != VisionPortal.CameraState.STREAMING) return;
        ExposureControl exposure = tracker.portal().getCameraControl(ExposureControl.class);
        GainControl gainControl = tracker.portal().getCameraControl(GainControl.class);
        if (exposure == null || gainControl == null) return;

        if (!cameraControlsRead) {
            exposureMs = exposure.getExposure(TimeUnit.MILLISECONDS);
            minExposureMs = exposure.getMinExposure(TimeUnit.MILLISECONDS);
            maxExposureMs = exposure.getMaxExposure(TimeUnit.MILLISECONDS);
            gain = gainControl.getGain();
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
            cameraControlsRead = true;
        }

        boolean changed = false;
        if (gamepad1.dpad_up && !lastUp) { exposureMs = Math.min(exposureMs + 1, maxExposureMs); changed = true; }
        if (gamepad1.dpad_down && !lastDown) { exposureMs = Math.max(exposureMs - 1, Math.max(minExposureMs, 1)); changed = true; }
        if (gamepad1.dpad_right && !lastRight) { gain = Math.min(gain + 10, maxGain); changed = true; }
        if (gamepad1.dpad_left && !lastLeft) { gain = Math.max(gain - 10, minGain); changed = true; }
        if (changed) {
            if (exposure.getMode() != ExposureControl.Mode.Manual) exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            gainControl.setGain(gain);
        }
        if (gamepad1.a && !lastA) {
            decimationIndex = (decimationIndex + 1) % DECIMATIONS.length;
            tracker.setDecimation(DECIMATIONS[decimationIndex]);
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastA = gamepad1.a;
    }

    @Override
    public void stop() {
        if (tracker != null) tracker.close();
    }
}
