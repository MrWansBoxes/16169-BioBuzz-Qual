package org.firstinspires.ftc.teamcode.subsystems.pollen;

import static org.firstinspires.ftc.teamcode.subsystems.pollen.PollenConstants.*;

import android.util.Size;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.turret.PoseHistory;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

/**
 * Finds POLLEN on the floor with the front camera. Other teams put yellow on their robots to fool cameras, so a
 * yellow blob only counts after five stages:
 * <ol>
 *   <li>yellow, round, and below the floor line (camera thread, {@link PollenProcessor});</li>
 *   <li>the right size for the floor distance its position implies (camera thread);</li>
 *   <li>up close, the ball's holes are visible (camera thread);</li>
 *   <li>its field position stays put for a few frames (here);</li>
 *   <li>it is inside the field walls and not inside a HIVE (here).</li>
 * </ol>
 * Positions come from projecting the blob center onto the floor through the camera's mounting geometry (a ball's
 * center sits at a known height), combined with the robot pose when the frame was captured. POLLEN that is not
 * on the floor (e.g. on a FLOWER) is rejected by design. Lens distortion is ignored.
 *
 * Used by the TeleOp assist (turn toward it) and by {@link PollenCommands#collectNearest} in auto.
 */
public class PollenTracker {
    /** Why a candidate was or wasn't accepted. The number is the stage that decided. */
    public enum Verdict {
        ACCEPTED("POLLEN"),
        CONFIRMING("4: not still yet"),
        AREA("1: area"),
        ROUNDNESS("1: not round"),
        ABOVE_FLOOR("1: above floor"),
        TOO_FAR("1: too far"),
        SIZE("2: wrong size"),
        HOLES("3: no holes"),
        OUTSIDE_FIELD("5: outside field"),
        IN_HIVE("5: in a HIVE");

        public final String label;

        Verdict(String label) {
            this.label = label;
        }
    }

    /** One yellow blob from one frame, with what each stage found. */
    public static final class Candidate {
        /** Blob circle in image pixels. */
        public final double u, v, radiusPx;
        public final int areaPx;
        public final double circularity;
        /** Ball center ahead of / left of the robot center at capture time, NaN if it can't be on the floor. */
        public final double forward, left;
        /** Measured / expected diameter (NaN if not checked). */
        public final double sizeRatio;
        /** Holes found (-1 if not checked: too far, or rejected earlier). */
        public final int holes;
        /** Stage 1-3 result, null if all passed. */
        final Verdict imageVerdict;
        /** Final result, set once stages 4-5 have run. */
        Verdict verdict;
        double fieldX = Double.NaN, fieldY = Double.NaN;

        Candidate(double u, double v, double radiusPx, int areaPx, double circularity, double[] floor,
                  double sizeRatio, int holes, Verdict imageVerdict) {
            this.u = u;
            this.v = v;
            this.radiusPx = radiusPx;
            this.areaPx = areaPx;
            this.circularity = circularity;
            this.forward = floor == null ? Double.NaN : floor[0];
            this.left = floor == null ? Double.NaN : floor[1];
            this.sizeRatio = sizeRatio;
            this.holes = holes;
            this.imageVerdict = imageVerdict;
            this.verdict = imageVerdict;
        }

        public Verdict verdict() {
            return verdict;
        }
    }

    /** One POLLEN position, relative to the robot's current pose. */
    public static final class Sighting {
        /** Ball center ahead of / left of the robot center. */
        public final double forward, left;
        /** Horizontal distance from the robot center and bearing (radians, + = left of straight ahead). */
        public final double range, bearing;
        /** Ball center on the field (Pedro coordinates). */
        public final double fieldX, fieldY;

        Sighting(double fieldX, double fieldY, Pose pose) {
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            double dx = fieldX - pose.x(), dy = fieldY - pose.y();
            double cos = Math.cos(pose.heading()), sin = Math.sin(pose.heading());
            this.forward = dx * cos + dy * sin;
            this.left = -dx * sin + dy * cos;
            this.range = Math.hypot(forward, left);
            this.bearing = Math.atan2(left, forward);
        }
    }

    /** A spot on the field where candidates keep appearing (stage 4). */
    private static final class Track {
        final double anchorX, anchorY;
        double x, y;
        int frames = 1;
        long lastSeenNanos;

        Track(double x, double y, long now) {
            this.anchorX = this.x = x;
            this.anchorY = this.y = y;
            this.lastSeenNanos = now;
        }

        boolean confirmed() {
            return frames >= STILL_FRAMES;
        }
    }

    private final PollenProcessor processor;
    private final VisionPortal portal;
    private final PoseHistory history = new PoseHistory(32);
    private final List<Track> tracks = new ArrayList<>();
    private long lastFrameId = 0;
    private List<Candidate> candidates = Collections.emptyList();
    private Sighting nearest = null;

    /** @param liveViewContainerId 0 for no preview, -1 for the normal preview, or a multi-portal view id. */
    public PollenTracker(HardwareMap hardwareMap, int liveViewContainerId) {
        processor = new PollenProcessor();
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

    /** Run stages 4-5 on any new camera frame and pick the nearest POLLEN. Call once per loop. */
    public void update(Pose pose) {
        long now = System.nanoTime();
        history.add(now, pose, 0, 0);

        PollenProcessor.Frame frame = processor.latest();
        if (frame != null && frame.id != lastFrameId) {
            lastFrameId = frame.id;
            handleFrame(frame, now);
        }

        Track best = null;
        double bestRange = Double.POSITIVE_INFINITY;
        for (Iterator<Track> it = tracks.iterator(); it.hasNext(); ) {
            Track t = it.next();
            if ((now - t.lastSeenNanos) / 1e9 > LOST_AFTER_SEC) {
                it.remove();
                continue;
            }
            double range = Math.hypot(t.x - pose.x(), t.y - pose.y());
            if (t.confirmed() && range < bestRange) {
                best = t;
                bestRange = range;
            }
        }
        nearest = best == null ? null : new Sighting(best.x, best.y, pose);
    }

    private void handleFrame(PollenProcessor.Frame frame, long now) {
        Pose then = history.at(frameTime(frame.captureNanos, now)).pose;
        List<Track> matchedThisFrame = new ArrayList<>();
        for (Candidate c : frame.candidates) {
            if (c.imageVerdict != null) continue;
            double cos = Math.cos(then.heading()), sin = Math.sin(then.heading());
            c.fieldX = then.x() + c.forward * cos - c.left * sin;
            c.fieldY = then.y() + c.forward * sin + c.left * cos;

            if (!insideField(c.fieldX, c.fieldY)) {
                c.verdict = Verdict.OUTSIDE_FIELD;
                continue;
            }
            if (insideHive(c.fieldX, c.fieldY)) {
                c.verdict = Verdict.IN_HIVE;
                continue;
            }

            Track track = null;
            double bestDistance = STILL_RADIUS_IN;
            for (Track t : tracks) {
                double d = Math.hypot(c.fieldX - t.anchorX, c.fieldY - t.anchorY);
                if (d <= bestDistance && !matchedThisFrame.contains(t)) {
                    track = t;
                    bestDistance = d;
                }
            }
            if (track == null) {
                track = new Track(c.fieldX, c.fieldY, now);
                tracks.add(track);
            } else {
                track.frames++;
                track.x = c.fieldX;
                track.y = c.fieldY;
                track.lastSeenNanos = now;
            }
            matchedThisFrame.add(track);
            c.verdict = track.confirmed() ? Verdict.ACCEPTED : Verdict.CONFIRMING;
        }
        candidates = frame.candidates;
    }

    private static boolean insideField(double x, double y) {
        return x >= FIELD_MARGIN_IN && x <= FIELD_SIZE - FIELD_MARGIN_IN
                && y >= FIELD_MARGIN_IN && y <= FIELD_SIZE - FIELD_MARGIN_IN;
    }

    /** Within HIVE_KEEPOUT_RADIUS of either HIVE center (same HIVE geometry as the turret uses). */
    private static boolean insideHive(double x, double y) {
        if (HIVE_KEEPOUT_RADIUS <= 0) return false;
        double far = Math.toRadians(TurretConstants.FAR_WALL_DIRECTION_DEG);
        // Facing the field from the audience, blue is on the right: "far" rotated by -90 degrees.
        double blueX = Math.sin(far), blueY = -Math.cos(far);
        for (int side = -1; side <= 1; side += 2) {
            double hx = TurretConstants.FIELD_CENTER_X + side * TurretConstants.HIVE_SIDE_OFFSET * blueX;
            double hy = TurretConstants.FIELD_CENTER_Y + side * TurretConstants.HIVE_SIDE_OFFSET * blueY;
            if (Math.hypot(x - hx, y - hy) <= HIVE_KEEPOUT_RADIUS) return true;
        }
        return false;
    }

    private static long frameTime(long frameNanos, long now) {
        long age = now - frameNanos;
        if (frameNanos == 0 || age < 0 || age > 500_000_000L) {
            return now - (long) (CAMERA_LATENCY_SEC * 1e9);
        }
        return frameNanos;
    }

    /**
     * Robot-frame {forward, left, depth} of a ball whose center is at pixel (u, v), where depth is the distance
     * along the camera's optical axis. Null if that pixel looks at or above the height of a ball center (it can't
     * be POLLEN on the floor).
     */
    public static double[] project(double u, double v) {
        double x = (u - CX) / FX;   // right in the image
        double y = (v - CY) / FY;   // down in the image
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        // Ray through the pixel, in the robot frame (forward, left, up), scaled so its optical-axis part is 1.
        double rayForward = Math.cos(pitch) + y * Math.sin(pitch);
        double rayLeft = -x;
        double rayUp = Math.sin(pitch) - y * Math.cos(pitch);
        double drop = POLLEN_DIAMETER / 2 - CAMERA_HEIGHT;
        if (rayUp >= -1e-6 || drop >= 0) return null;
        double t = drop / rayUp;
        return new double[]{CAMERA_FORWARD + t * rayForward, CAMERA_LEFT + t * rayLeft, t};
    }

    public boolean hasTarget() {
        return nearest != null;
    }

    /** The nearest accepted POLLEN, relative to the pose last passed to {@link #update}, or null. */
    public Sighting nearest() {
        return nearest;
    }

    /** Every candidate from the most recent frame, with its verdict (for tuning). */
    public List<Candidate> candidates() {
        return candidates;
    }

    public VisionPortal portal() {
        return portal;
    }

    public void close() {
        portal.close();
    }

    public void addTelemetry(Telemetry telemetry) {
        int accepted = 0;
        for (Candidate c : candidates) if (c.verdict == Verdict.ACCEPTED) accepted++;
        Sighting s = nearest();
        if (s == null) {
            telemetry.addData("POLLEN", "none, %d yellow candidates (%s)", candidates.size(), portal.getCameraState());
        } else {
            telemetry.addData("POLLEN", "%d of %d candidates, nearest %.1f in at %.0f deg, field (%.1f, %.1f)",
                    accepted, candidates.size(), s.range, Math.toDegrees(s.bearing), s.fieldX, s.fieldY);
        }
    }
}
