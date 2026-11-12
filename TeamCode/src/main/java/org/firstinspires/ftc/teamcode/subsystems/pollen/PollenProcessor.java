package org.firstinspires.ftc.teamcode.subsystems.pollen;

import static org.firstinspires.ftc.teamcode.subsystems.pollen.PollenConstants.*;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.pollen.PollenTracker.Candidate;
import org.firstinspires.ftc.teamcode.subsystems.pollen.PollenTracker.Verdict;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * POLLEN stages 1-3, run on the camera thread because the hole check needs the image:
 *  1. yellow blobs (SDK {@link ColorBlobLocatorProcessor}) below the floor line, of plausible area and roundness;
 *  2. blob size matches a ball at the floor distance its position implies;
 *  3. up close, darker round spots (the holes) inside the blob.
 * Stages 4-5 need the robot pose and run in {@link PollenTracker#update}.
 *
 * On the Driver Station camera stream: cyan box = search area, green circle = passed stages 1-3,
 * red circle + label = rejected (the label names the stage).
 */
final class PollenProcessor implements VisionProcessor {
    /** One processed camera frame. */
    static final class Frame {
        final long id;
        final long captureNanos;
        final List<Candidate> candidates;

        Frame(long id, long captureNanos, List<Candidate> candidates) {
            this.id = id;
            this.captureNanos = captureNanos;
            this.candidates = candidates;
        }
    }

    /** What onDrawFrame gets for each frame: the SDK processor's own context plus our candidates. */
    private static final class DrawContext {
        final Object blobContext;
        final List<Candidate> candidates;

        DrawContext(Object blobContext, List<Candidate> candidates) {
            this.blobContext = blobContext;
            this.candidates = candidates;
        }
    }

    private final ColorBlobLocatorProcessor blobs;
    private volatile Frame latest = null;
    private long nextId = 1;
    private Paint passPaint, rejectPaint, textPaint;

    PollenProcessor() {
        blobs = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(COLOR)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asImageCoordinates(0, floorLineRow(), CAMERA_WIDTH, CAMERA_HEIGHT_PX))
                .setDrawContours(false)
                .setBoxFitColor(0)
                .setCircleFitColor(0)
                .setRoiColor(Color.CYAN)
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
    }

    /**
     * Top image row of the search area: where the top of a ball MAX_RANGE straight ahead appears, minus
     * ROI_MARGIN_PX. A ball on the floor any closer appears lower in the image.
     */
    static int floorLineRow() {
        double elevation = Math.atan2(POLLEN_DIAMETER - CAMERA_HEIGHT, MAX_RANGE - CAMERA_FORWARD);
        double belowAxis = Math.toRadians(CAMERA_PITCH_DEG) - elevation;
        int row = (int) Math.floor(CY + FY * Math.tan(belowAxis)) - ROI_MARGIN_PX;
        return Math.max(0, Math.min(row, CAMERA_HEIGHT_PX - 20));
    }

    /** The most recent processed frame, or null before the first one. */
    Frame latest() {
        return latest;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        blobs.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Object blobContext = blobs.processFrame(frame, captureTimeNanos);
        List<Candidate> candidates = new ArrayList<>();
        for (ColorBlobLocatorProcessor.Blob blob : blobs.getBlobs()) {
            // Tiny specks aren't worth listing; real rejections start at a quarter of MIN_AREA_PX.
            if (blob.getContourArea() < MIN_AREA_PX / 4) continue;
            candidates.add(evaluate(frame, blob));
        }
        candidates = Collections.unmodifiableList(candidates);
        latest = new Frame(nextId++, captureTimeNanos, candidates);
        return new DrawContext(blobContext, candidates);
    }

    private static Candidate evaluate(Mat frame, ColorBlobLocatorProcessor.Blob blob) {
        Circle circle = blob.getCircle();
        double u = circle.getX(), v = circle.getY(), radius = circle.getRadius();
        int area = blob.getContourArea();
        double circularity = blob.getCircularity();

        if (area < MIN_AREA_PX || area > MAX_AREA_PX) {
            return new Candidate(u, v, radius, area, circularity, null, Double.NaN, -1, Verdict.AREA);
        }
        if (circularity < MIN_CIRCULARITY) {
            return new Candidate(u, v, radius, area, circularity, null, Double.NaN, -1, Verdict.ROUNDNESS);
        }
        double[] floor = PollenTracker.project(u, v);
        if (floor == null) {
            return new Candidate(u, v, radius, area, circularity, null, Double.NaN, -1, Verdict.ABOVE_FLOOR);
        }
        if (Math.hypot(floor[0], floor[1]) > MAX_RANGE) {
            return new Candidate(u, v, radius, area, circularity, floor, Double.NaN, -1, Verdict.TOO_FAR);
        }
        // A ball at depth z (along the optical axis) is about FX * diameter / z pixels across.
        double sizeRatio = 2 * radius / (FX * POLLEN_DIAMETER / floor[2]);
        if (Math.abs(sizeRatio - 1) > SIZE_TOLERANCE) {
            return new Candidate(u, v, radius, area, circularity, floor, sizeRatio, -1, Verdict.SIZE);
        }
        int holes = -1;
        if (Math.hypot(floor[0], floor[1]) <= HOLE_CHECK_RANGE_IN) {
            holes = countHoles(frame, u, v, radius);
            if (holes < MIN_HOLES) {
                return new Candidate(u, v, radius, area, circularity, floor, sizeRatio, holes, Verdict.HOLES);
            }
        }
        return new Candidate(u, v, radius, area, circularity, floor, sizeRatio, holes, null);
    }

    /**
     * Number of dark, hole-sized spots inside the ball. The ball is yellow inside too, so a hole shows as a
     * patch noticeably darker than the ball's typical brightness, not as a different color. Patches much bigger
     * than a hole (the shaded underside) or much smaller (noise) don't count.
     */
    static int countHoles(Mat rgb, double cx, double cy, double radius) {
        double inner = HOLE_SEARCH_FRACTION * radius;
        int left = (int) Math.max(0, Math.floor(cx - inner));
        int top = (int) Math.max(0, Math.floor(cy - inner));
        int right = (int) Math.min(rgb.cols(), Math.ceil(cx + inner));
        int bottom = (int) Math.min(rgb.rows(), Math.ceil(cy + inner));
        int w = right - left, h = bottom - top;
        if (w < 8 || h < 8) return 0;

        Mat gray = new Mat();
        Mat mask = new Mat(h, w, CvType.CV_8UC1);
        Mat labels = new Mat(), stats = new Mat(), centroids = new Mat();
        try {
            Mat sub = rgb.submat(new Rect(left, top, w, h));
            Imgproc.cvtColor(sub, gray, Imgproc.COLOR_RGB2GRAY);
            sub.release();
            byte[] pixels = new byte[w * h];
            gray.get(0, 0, pixels);

            // Median brightness inside the search disc.
            boolean[] inDisc = new boolean[w * h];
            int[] histogram = new int[256];
            int count = 0;
            double inner2 = inner * inner;
            for (int y = 0; y < h; y++) {
                double dy = top + y + 0.5 - cy;
                for (int x = 0; x < w; x++) {
                    double dx = left + x + 0.5 - cx;
                    if (dx * dx + dy * dy <= inner2) {
                        inDisc[y * w + x] = true;
                        histogram[pixels[y * w + x] & 0xFF]++;
                        count++;
                    }
                }
            }
            if (count == 0) return 0;
            int median = 0;
            for (int seen = 0; median < 255; median++) {
                seen += histogram[median];
                if (seen * 2 >= count) break;
            }
            double threshold = HOLE_DARKNESS * median;

            byte[] dark = new byte[w * h];
            for (int i = 0; i < dark.length; i++) {
                if (inDisc[i] && (pixels[i] & 0xFF) < threshold) dark[i] = (byte) 255;
            }
            mask.put(0, 0, dark);

            int n = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8, CvType.CV_32S);
            double holeArea = Math.PI * Math.pow(HOLE_RADIUS_RATIO * radius, 2);
            double minArea = Math.max(4, 0.2 * holeArea), maxArea = 2.5 * holeArea;
            int holes = 0;
            for (int i = 1; i < n; i++) {   // label 0 is the background
                double a = stats.get(i, Imgproc.CC_STAT_AREA)[0];
                if (a >= minArea && a <= maxArea) holes++;
            }
            return holes;
        } finally {
            gray.release();
            mask.release();
            labels.release();
            stats.release();
            centroids.release();
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext) {
        if (!(userContext instanceof DrawContext)) return;
        DrawContext context = (DrawContext) userContext;
        blobs.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity,
                context.blobContext);

        if (passPaint == null) {
            passPaint = stroke(Color.GREEN, scaleCanvasDensity);
            rejectPaint = stroke(Color.RED, scaleCanvasDensity);
            textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
            textPaint.setColor(Color.WHITE);
            textPaint.setTextSize(14 * scaleCanvasDensity);
        }
        float s = scaleBmpPxToCanvasPx;
        for (Candidate c : context.candidates) {
            boolean passed = c.imageVerdict == null;
            float x = (float) c.u * s, y = (float) c.v * s, r = (float) c.radiusPx * s;
            canvas.drawCircle(x, y, r, passed ? passPaint : rejectPaint);
            String label = passed ? (c.holes >= 0 ? "1-3 ok, " + c.holes + " holes" : "1-3 ok") : c.imageVerdict.label;
            canvas.drawText(label, x - r, y - r - 4 * scaleCanvasDensity, textPaint);
        }
    }

    private static Paint stroke(int color, float density) {
        Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        paint.setColor(color);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(3 * density);
        return paint;
    }
}
