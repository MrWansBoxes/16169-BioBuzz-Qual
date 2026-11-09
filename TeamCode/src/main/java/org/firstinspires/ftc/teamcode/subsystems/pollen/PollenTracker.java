package org.firstinspires.ftc.teamcode.subsystems.pollen;

import static org.firstinspires.ftc.teamcode.subsystems.pollen.PollenConstants.*;

import android.graphics.Color;
import android.util.Size;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

/**
 * Finds the nearest POLLEN with the front camera. Yellow blobs that are round enough are projected onto the
 * floor through the camera's mounting geometry (a ball's center sits at a known height), which gives a position
 * relative to the robot and, with the robot pose, on the field.
 *
 * Used by the TeleOp assist (turn toward it) and by {@link PollenCommands#collectNearest} in auto.
 * Lens distortion is ignored; the goBILDA lens is mild and the intake funnel absorbs a few inches of error.
 */
public class PollenTracker {
    /** One POLLEN position. */
    public static final class Sighting {
        /** Ball center ahead of / left of the robot center. */
        public final double forward, left;
        /** Horizontal distance from the robot center and bearing (radians, + = left of straight ahead). */
        public final double range, bearing;
        /** Ball center on the field (Pedro coordinates). */
        public final double fieldX, fieldY;

        Sighting(double forward, double left, Pose pose) {
            this.forward = forward;
            this.left = left;
            this.range = Math.hypot(forward, left);
            this.bearing = Math.atan2(left, forward);
            double cos = Math.cos(pose.heading()), sin = Math.sin(pose.heading());
            this.fieldX = pose.x() + forward * cos - left * sin;
            this.fieldY = pose.y() + forward * sin + left * cos;
        }
    }

    private final ColorBlobLocatorProcessor processor;
    private final VisionPortal portal;
    private Sighting nearest = null;
    private int candidates = 0;
    private long lastSeenNanos = 0;

    /** @param liveViewContainerId 0 for no preview, -1 for the normal preview, or a multi-portal view id. */
    public PollenTracker(HardwareMap hardwareMap, int liveViewContainerId) {
        processor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(COLOR)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.MAGENTA)
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

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

    /** Pick the nearest POLLEN in the latest frame. Call once per loop with the current robot pose. */
    public void update(Pose pose) {
        List<ColorBlobLocatorProcessor.Blob> blobs = processor.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, MIN_AREA_PX, MAX_AREA_PX, blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, MIN_CIRCULARITY, 1, blobs);

        Sighting best = null;
        candidates = 0;
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            Circle circle = blob.getCircle();
            double[] floor = robotFramePosition(circle.getX(), circle.getY());
            if (floor == null) continue;
            Sighting s = new Sighting(floor[0], floor[1], pose);
            if (s.range > MAX_RANGE) continue;
            candidates++;
            if (best == null || s.range < best.range) best = s;
        }
        if (best != null) {
            nearest = best;
            lastSeenNanos = System.nanoTime();
        }
    }

    /**
     * Robot-frame {forward, left} of a ball whose center is at pixel (u, v), or null if that pixel looks at or
     * above the height of a ball center (it can't be POLLEN on the floor).
     */
    public static double[] robotFramePosition(double u, double v) {
        double x = (u - CX) / FX;   // right in the image
        double y = (v - CY) / FY;   // down in the image
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        // Ray through the pixel, in the robot frame (forward, left, up).
        double rayForward = Math.cos(pitch) + y * Math.sin(pitch);
        double rayLeft = -x;
        double rayUp = Math.sin(pitch) - y * Math.cos(pitch);
        double drop = POLLEN_DIAMETER / 2 - CAMERA_HEIGHT;
        if (rayUp >= -1e-6 || drop >= 0) return null;
        double t = drop / rayUp;
        return new double[]{CAMERA_FORWARD + t * rayForward, CAMERA_LEFT + t * rayLeft};
    }

    public boolean hasTarget() {
        return nearest != null && (System.nanoTime() - lastSeenNanos) / 1e9 < LOST_AFTER_SEC;
    }

    /** The nearest POLLEN, or null if nothing has been seen recently. */
    public Sighting nearest() {
        return hasTarget() ? nearest : null;
    }

    public VisionPortal portal() {
        return portal;
    }

    public void close() {
        portal.close();
    }

    public void addTelemetry(Telemetry telemetry) {
        Sighting s = nearest();
        if (s == null) {
            telemetry.addData("POLLEN", "none (%s)", portal.getCameraState());
        } else {
            telemetry.addData("POLLEN", "%d seen, nearest %.1f in at %.0f deg, field (%.1f, %.1f)",
                    candidates, s.range, Math.toDegrees(s.bearing), s.fieldX, s.fieldY);
        }
    }
}
