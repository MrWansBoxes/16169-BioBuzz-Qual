package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.pollen.PollenTracker;

/**
 * POLLEN camera bring-up: needs ONLY the front webcam (configured as "PollenCam").
 *
 * What to test (robot on the tiles, camera mounted):
 *  1. Detection: open the Driver Station camera stream (3-dot menu > Camera Stream, during INIT). Every POLLEN
 *     should get a magenta circle; nothing else should. If the field or lights also show up, tighten
 *     PollenConstants.COLOR / MIN_CIRCULARITY / MIN_AREA_PX.
 *  2. Distance: put one POLLEN straight ahead, 24 in. from the robot center. "forward" should read ~24 and
 *     "left" ~0. Move it 12 in. farther: forward should grow by ~12. If it is off by a constant factor,
 *     check CAMERA_HEIGHT and CAMERA_PITCH_DEG; if only far balls are off, the pitch is wrong.
 *  3. Side: move it 12 in. to the robot's left: "left" should read ~+12.
 */
@TeleOp(name = "POLLEN Camera Test", group = "Test")
public class PollenCameraTest extends OpMode {
    private static final Pose ORIGIN = new Pose(0, 0, 0);

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
        pollen.update(ORIGIN);
        telemetry.addData("Camera", "%s  %.0f fps", pollen.portal().getCameraState(), pollen.portal().getFps());
        PollenTracker.Sighting s = pollen.nearest();
        if (s == null) {
            telemetry.addLine("No POLLEN");
        } else {
            telemetry.addData("Nearest POLLEN (robot frame)", "forward %.1f  left %.1f in", s.forward, s.left);
            telemetry.addData("Range / bearing", "%.1f in / %.1f deg (+ = left)", s.range, Math.toDegrees(s.bearing));
        }
        pollen.addTelemetry(telemetry);
    }

    @Override
    public void stop() {
        if (pollen != null) pollen.close();
    }
}
