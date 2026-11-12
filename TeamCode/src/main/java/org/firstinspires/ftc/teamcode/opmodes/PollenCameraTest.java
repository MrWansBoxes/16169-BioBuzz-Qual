package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.pollen.PollenTracker;

import java.util.List;

/**
 * POLLEN camera bring-up: needs ONLY the front webcam (configured as "PollenCam"). The robot is assumed to sit
 * still at {@link #TEST_POSE}, so the field checks (stage 5) have room ahead of it.
 *
 * Camera stream (3-dot menu > Camera Stream, during INIT): cyan box = search area, green circle = passed stages
 * 1-3, red circle = rejected, labelled with the stage. Telemetry lists every candidate with its final verdict.
 *
 * What to test (robot on the tiles, camera mounted):
 *  1. Detection: every POLLEN gets a green circle and reads "POLLEN" in telemetry; yellow on robots doesn't.
 *  2. Distance: put one POLLEN straight ahead, 24 in. from the robot center. "forward" should read ~24 and
 *     "left" ~0. Move it 12 in. farther: forward should grow by ~12. If it is off by a constant factor,
 *     check CAMERA_HEIGHT and POLLEN_DIAMETER; if only far balls are off, the pitch is wrong.
 *  3. Side: move it 12 in. to the robot's left: "left" should read ~+12.
 *  4. Size and holes: the "size" ratio should stay near 1.00 at every distance, and a ball within
 *     HOLE_CHECK_RANGE_IN should show at least MIN_HOLES holes in any orientation.
 */
@TeleOp(name = "POLLEN Camera Test", group = "Test")
public class PollenCameraTest extends OpMode {
    /** Audience-side tile row, centered, facing the far wall (same as TurretTrackingTest). */
    public static Pose TEST_POSE = new Pose(72, 24, Math.toRadians(90));

    private PollenTracker pollen;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(100);
        pollen = new PollenTracker(hardwareMap, -1);
    }

    @Override
    public void init_loop() {
        show();
    }

    @Override
    public void loop() {
        show();
    }

    private void show() {
        pollen.update(TEST_POSE);
        telemetry.addData("Camera", "%s  %.0f fps", pollen.portal().getCameraState(), pollen.portal().getFps());
        PollenTracker.Sighting s = pollen.nearest();
        if (s == null) {
            telemetry.addLine("No POLLEN accepted");
        } else {
            telemetry.addData("Nearest POLLEN (robot frame)", "forward %.1f  left %.1f in", s.forward, s.left);
            telemetry.addData("Range / bearing", "%.1f in / %.1f deg (+ = left)", s.range, Math.toDegrees(s.bearing));
        }

        List<PollenTracker.Candidate> candidates = pollen.candidates();
        telemetry.addLine(String.format("\n==== Yellow candidates: %d", candidates.size()));
        for (PollenTracker.Candidate c : candidates) {
            PollenTracker.Verdict verdict = c.verdict();
            telemetry.addLine(String.format("-- %s  at px (%.0f, %.0f) r %.0f",
                    verdict == null ? "?" : verdict.label, c.u, c.v, c.radiusPx));
            telemetry.addLine(String.format("   area %d px  round %.2f  fwd %.1f  left %.1f in",
                    c.areaPx, c.circularity, c.forward, c.left));
            telemetry.addLine(String.format("   size %.2f  holes %s",
                    c.sizeRatio, c.holes < 0 ? "not checked" : Integer.toString(c.holes)));
        }
    }

    @Override
    public void stop() {
        if (pollen != null) pollen.close();
    }
}
