package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.*;

import android.util.Size;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Knows where our alliance's raised CELL is in field coordinates.
 *
 * BIOBUZZ facts this relies on (manual section 9.6 / 9.9):
 *  - The HIVE frame is anchored at field center, so each CELL can only be in one of two known places
 *    (raised or lowered). {@link #priorPosition} computes those from {@link TurretConstants}.
 *  - Each CELL has an AprilTag cluster on its underside; the SDK reports the cluster origin, which is the
 *    center of the CELL opening, i.e. exactly what we want to shoot at.
 *  - The SDK cannot localize the robot from these tags (it gives every cluster a field position of 0,0,0
 *    because they tip), so instead we measure the CELL relative to the robot and compare it with where
 *    odometry says it should be. That difference ({@link #correctionX}/{@link #correctionY}) is mostly
 *    odometry drift; it is low-pass filtered and added to the aim point, and it keeps working after the
 *    camera loses sight of the HIVE.
 *  - A sighted CELL's measured height tells us whether it is raised (~59 in.) or lowered (~44 in.). Seeing
 *    either of our CELLs is enough to know which one is raised.
 */
public class HiveTracker {
    public enum Cell { SCORING, AUDIENCE }

    /** Pass as {@code liveViewContainerId} to show the preview in the default Robot Controller view. */
    public static final int DEFAULT_LIVE_VIEW = -1;

    /** One AprilTag detection converted to the turret frame (inches). */
    public static final class Measurement {
        /** Horizontal distance ahead of / left of the turret pivot. */
        public final double forward, left;
        /** Height above the floor. */
        public final double height;
        /** Horizontal distance from the pivot and bearing (radians, + = left of the turret's pointing direction). */
        public final double range, bearing;

        Measurement(double forward, double left, double height) {
            this.forward = forward;
            this.left = left;
            this.height = height;
            this.range = Math.hypot(forward, left);
            this.bearing = Math.atan2(left, forward);
        }
    }

    private Alliance alliance;
    private final AprilTagProcessor processor;
    private final VisionPortal portal;

    private Cell raised;
    private boolean raisedConfirmedByCamera = false;
    private Cell pendingRaised = null;
    private int pendingRaisedFrames = 0;

    private boolean hasCorrection = false;
    private double correctionX = 0, correctionY = 0;
    private int outlierFrames = 0;

    private long lastSeenNanos = 0;
    private Cell lastSeenCell = null;
    private double lastSeenHeight = Double.NaN;
    private int lastPercentFound = 0;
    private List<AprilTagDetection> latestDetections = new ArrayList<>();
    private PoseHistory stationaryHistory;

    /**
     * @param liveViewContainerId 0 for no preview, {@link #DEFAULT_LIVE_VIEW} for the normal preview. With two
     *                            cameras, pass one of the ids from {@code VisionPortal.makeMultiPortalView(2, ...)}
     *                            so both previews fit.
     */
    public HiveTracker(HardwareMap hardwareMap, Alliance alliance, int liveViewContainerId) {
        this.alliance = alliance;
        this.raised = startingRaisedCell(alliance);

        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();
        processor.setDecimation(APRILTAG_DECIMATION);

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT_PX))
                // MJPEG keeps USB bandwidth low enough for two cameras on one hub.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor);
        if (liveViewContainerId > 0) {
            builder.setLiveViewContainerId(liveViewContainerId);
        } else if (liveViewContainerId == 0) {
            builder.enableLiveView(false);
        }
        portal = builder.build();
    }

    /**
     * Which CELL is raised at match start. Manual 10.3.1: "the CELL which points at a FLOWER should be the one
     * tilted down", which leaves red's AUDIENCE CELL and blue's SCORING CELL raised.
     */
    public static Cell startingRaisedCell(Alliance alliance) {
        return alliance == Alliance.RED ? Cell.AUDIENCE : Cell.SCORING;
    }

    /**
     * Continue from what an earlier OpMode (auto) learned: which CELL is raised and the odometry correction.
     * {@code cameraConfirmed} = the camera had actually seen the HIVE, so a tip needs several frames to accept.
     */
    public void restore(Cell raisedCell, double correctionX, double correctionY, boolean cameraConfirmed) {
        setRaisedCell(raisedCell);
        this.correctionX = correctionX;
        this.correctionY = correctionY;
        this.hasCorrection = true;
        this.raisedConfirmedByCamera = cameraConfirmed;
    }

    /** Switch alliance (e.g. from an init menu). Forgets everything learned so far. */
    public void setAlliance(Alliance newAlliance) {
        alliance = newAlliance;
        raised = startingRaisedCell(newAlliance);
        raisedConfirmedByCamera = false;
        pendingRaised = null;
        pendingRaisedFrames = 0;
        hasCorrection = false;
        correctionX = 0;
        correctionY = 0;
        outlierFrames = 0;
        lastSeenNanos = 0;
        lastSeenCell = null;
        lastSeenHeight = Double.NaN;
    }

    public Alliance alliance() {
        return alliance;
    }

    /** Tell the tracker which CELL is raised before the camera has seen one (e.g. chosen in init). */
    public void setRaisedCell(Cell cell) {
        raised = cell;
        pendingRaised = null;
        pendingRaisedFrames = 0;
    }

    /** Process any new camera frame. Call once per loop, after the newest pose was added to {@code history}. */
    void update(PoseHistory history) {
        List<AprilTagDetection> detections = processor.getFreshDetections();
        if (detections == null) return;
        latestDetections = detections;

        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            Cell cell = cellFromName(cluster.metadata.name);
            if (cell != null) {
                handleSighting(cell, cluster, history);
            }
        }
    }

    /**
     * Convert a detection from the camera frame to the turret frame, with the camera tilted to {@code pitchRad}
     * (the tilt moves both the lens direction and, because it swings around the tilt axis, the lens position).
     * Public so bench tests can check the geometry against a tape measure.
     */
    public static Measurement measure(AprilTagDetection d, double pitchRad) {
        // ftcPose is in the camera frame: x right, y out of the lens, z up. Undo the camera tilt so we get
        // horizontal-forward and true-vertical components.
        double forward = d.ftcPose.y * Math.cos(pitchRad) - d.ftcPose.z * Math.sin(pitchRad);
        double up = d.ftcPose.y * Math.sin(pitchRad) + d.ftcPose.z * Math.cos(pitchRad);
        double left = -d.ftcPose.x;

        // Camera -> turret frame.
        double[] lens = CameraTilt.lensPosition(pitchRad);
        double camYaw = Math.toRadians(CAMERA_YAW_DEG);
        return new Measurement(
                lens[0] + forward * Math.cos(camYaw) - left * Math.sin(camYaw),
                lens[1] + forward * Math.sin(camYaw) + left * Math.cos(camYaw),
                lens[2] + up);
    }

    /** Raised / lowered from a measured CELL height, or null inside the ambiguous band around the threshold. */
    public static Boolean isRaisedHeight(double height) {
        if (height > RAISED_HEIGHT_THRESHOLD + RAISED_HEIGHT_DEADBAND) return true;
        if (height < RAISED_HEIGHT_THRESHOLD - RAISED_HEIGHT_DEADBAND) return false;
        return null;
    }

    private void handleSighting(Cell cell, AprilTagClusterDetection d, PoseHistory history) {
        PoseHistory.Sample then = history.at(frameTime(d.frameAcquisitionNanoTime));
        if (then == null) return;
        Measurement m = measure(d, then.cameraPitchRad);
        if (m.range > MAX_TRUSTED_RANGE) return;
        double turretForward = m.forward;
        double turretLeft = m.left;

        // Turret frame -> field, using where the robot and turret were when the frame was captured.
        double heading = then.pose.heading();
        double turretHeading = heading + then.turretRad;
        double pivotX = then.pose.x() + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double pivotY = then.pose.y() + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);
        double measuredX = pivotX + turretForward * Math.cos(turretHeading) - turretLeft * Math.sin(turretHeading);
        double measuredY = pivotY + turretForward * Math.sin(turretHeading) + turretLeft * Math.cos(turretHeading);
        double height = m.height;

        lastSeenNanos = System.nanoTime();
        lastSeenCell = cell;
        lastSeenHeight = height;
        lastPercentFound = d.percentClusterFound;

        // Near the threshold the HIVE is probably mid-tip; don't learn anything from that frame.
        Boolean raisedHeight = isRaisedHeight(height);
        if (raisedHeight == null) return;
        boolean isRaised = raisedHeight;

        updateRaised(isRaised ? cell : other(cell));
        updateCorrection(measuredX, measuredY, priorPosition(cell, isRaised), d.percentClusterFound);
    }

    private void updateRaised(Cell observedRaised) {
        if (!raisedConfirmedByCamera) {
            raised = observedRaised;
            raisedConfirmedByCamera = true;
            return;
        }
        if (observedRaised == raised) {
            pendingRaised = null;
            pendingRaisedFrames = 0;
            return;
        }
        if (observedRaised != pendingRaised) {
            pendingRaised = observedRaised;
            pendingRaisedFrames = 0;
        }
        if (++pendingRaisedFrames >= FRAMES_TO_CONFIRM_TIP) {
            raised = observedRaised;
            pendingRaised = null;
            pendingRaisedFrames = 0;
        }
    }

    private void updateCorrection(double measuredX, double measuredY, double[] prior, int percentFound) {
        double impliedX = measuredX - prior[0];
        double impliedY = measuredY - prior[1];
        if (!hasCorrection) {
            correctionX = impliedX;
            correctionY = impliedY;
            hasCorrection = true;
            return;
        }
        double dx = impliedX - correctionX;
        double dy = impliedY - correctionY;
        if (Math.hypot(dx, dy) > OUTLIER_DISTANCE) {
            if (++outlierFrames < OUTLIER_FRAMES_TO_RESET) return;
            correctionX = impliedX;
            correctionY = impliedY;
            outlierFrames = 0;
            return;
        }
        outlierFrames = 0;
        // Partially occluded clusters give noisier poses, so trust them less.
        double gain = FILTER_GAIN * Math.max(0.25, percentFound / 100.0);
        correctionX += gain * dx;
        correctionY += gain * dy;
    }

    /** Where a CELL opening should be in Pedro field coordinates, ignoring odometry error. */
    double[] priorPosition(Cell cell, boolean isRaised) {
        double far = Math.toRadians(FAR_WALL_DIRECTION_DEG);
        double farX = Math.cos(far), farY = Math.sin(far);
        // Facing the field from the audience, blue is on the right: rotate "far" by -90 degrees.
        double blueX = farY, blueY = -farX;
        double side = alliance == Alliance.BLUE ? 1 : -1;
        double end = cell == Cell.SCORING ? 1 : -1;
        double reach = isRaised ? RAISED_CELL_REACH : LOWERED_CELL_REACH;
        return new double[]{
                FIELD_CENTER_X + side * HIVE_SIDE_OFFSET * blueX + end * reach * farX,
                FIELD_CENTER_Y + side * HIVE_SIDE_OFFSET * blueY + end * reach * farY
        };
    }

    /** Field position (x, y) of the raised CELL's opening center, including the camera correction. */
    public double[] aimPoint() {
        double[] p = priorPosition(raised, true);
        return new double[]{p[0] + correctionX, p[1] + correctionY};
    }

    public double aimHeight() {
        return RAISED_CELL_HEIGHT;
    }

    public Cell raisedCell() {
        return raised;
    }

    /** False until the camera has seen one of our CELLs; until then the aim point is pure odometry + prior. */
    public boolean hasSeenHive() {
        return raisedConfirmedByCamera;
    }

    public double secondsSinceSeen() {
        return lastSeenNanos == 0 ? Double.POSITIVE_INFINITY : (System.nanoTime() - lastSeenNanos) / 1e9;
    }

    /**
     * Filtered (measured - expected) CELL position. Odometry error is roughly the negative of this, so it
     * could also be fed back into the follower's pose if the field turns out to be accurate enough.
     */
    public double[] correction() {
        return new double[]{correctionX, correctionY};
    }

    public Cell lastSeenCell() {
        return lastSeenCell;
    }

    public double lastSeenHeight() {
        return lastSeenHeight;
    }

    public int lastPercentFound() {
        return lastPercentFound;
    }

    /**
     * For testing without a turret or drivetrain: process new frames as if the robot sat still at {@code pose}
     * with the turret at {@code turretDegrees} and the camera at {@code pitchDegrees}.
     */
    public void updateStationary(Pose pose, double turretDegrees, double pitchDegrees) {
        if (stationaryHistory == null) stationaryHistory = new PoseHistory(8);
        stationaryHistory.add(System.nanoTime(), pose, Math.toRadians(turretDegrees), Math.toRadians(pitchDegrees));
        update(stationaryHistory);
    }

    /** Every detection (any tag or cluster, any alliance) from the most recent processed frame. */
    public List<AprilTagDetection> latestDetections() {
        return latestDetections;
    }

    public void setDecimation(float decimation) {
        processor.setDecimation(decimation);
    }

    public VisionPortal portal() {
        return portal;
    }

    public VisionPortal.CameraState cameraState() {
        return portal.getCameraState();
    }

    public void close() {
        portal.close();
    }

    private Cell cellFromName(String clusterName) {
        if (!alliance.ownsCluster(clusterName)) return null;
        if (clusterName.endsWith("SCORING")) return Cell.SCORING;
        if (clusterName.endsWith("AUDIENCE")) return Cell.AUDIENCE;
        return null;
    }

    private static Cell other(Cell cell) {
        return cell == Cell.SCORING ? Cell.AUDIENCE : Cell.SCORING;
    }

    /** The frame timestamp should be System.nanoTime(); fall back to a fixed latency if it looks wrong. */
    private static long frameTime(long frameNanos) {
        long now = System.nanoTime();
        long age = now - frameNanos;
        if (frameNanos == 0 || age < 0 || age > 500_000_000L) {
            return now - (long) (DEFAULT_CAMERA_LATENCY_SEC * 1e9);
        }
        return frameNanos;
    }
}
