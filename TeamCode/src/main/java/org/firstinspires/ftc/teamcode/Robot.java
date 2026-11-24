package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.CorrectedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.intake.Dropdown;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.pollen.PollenTracker;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.turret.HiveTracker;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

import java.util.function.Supplier;

/**
 * All mechanisms (not the drivetrain, which is Pedro's Follower), shared by auto and TeleOp.
 *
 * Robot configuration (Control Hub + Expansion Hub; every motor is a goBILDA bare motor with encoder, every
 * servo an Axon Mini). The ports are a proposal; only the names matter to the code.
 * <pre>
 *   Control Hub   motor 0-3   lf, lr, rf, rr                                   (Pedro drivetrain)
 *                 servo 0-2   turretServoA, turretServoB, cameraTilt
 *                 analog 0-1  turretFeedback (servo A's 4th wire), cameraTiltFeedback
 *                 I2C bus 1   pinpoint                                         (goBILDA Pinpoint)
 *   Expansion Hub motor 0-3   intakeFront, intakeTransfer, flywheelA, flywheelB
 *                 servo 0-1   dropdownLeft, dropdownRight
 *   USB                       TurretCam, PollenCam                             (goBILDA global shutter)
 * </pre>
 */
public class Robot {
    public final HiveTracker tracker;
    public final Turret turret;
    public final Flywheel flywheel;
    public final Intake intake;
    public final Dropdown dropdown;
    public final PollenTracker pollen;
    private final Supplier<Pose> pose;
    /** Null if Constants.create() doesn't wrap its localizer: then the camera only corrects the aim. */
    private final CorrectedLocalizer localizer;

    public Robot(HardwareMap hardwareMap, Alliance alliance, Follower follower) {
        this.pose = follower::pose;
        this.localizer = follower.localizer instanceof CorrectedLocalizer
                ? (CorrectedLocalizer) follower.localizer : null;
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        tracker = new HiveTracker(hardwareMap, alliance, 0);
        turret = new Turret(hardwareMap, pose, tracker);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        dropdown = new Dropdown(hardwareMap);
        pollen = new PollenTracker(hardwareMap, 0);
    }

    /** Ready to shoot: turret aimed and flywheel at speed. */
    public boolean readyToShoot() {
        return turret.isOnTarget() && flywheel.isReady();
    }

    /** Once per loop, after follower.update() and Scheduler.execute(). */
    public void update() {
        turret.update();
        if (localizer != null) {
            double[] shift = turret.correctPose(localizer);
            if (shift != null) pollen.shiftField(shift[0], shift[1]);
        }
        flywheel.update();
        intake.update();
        pollen.update(pose.get());
    }

    public void addTelemetry(Telemetry telemetry) {
        if (localizer == null) {
            telemetry.addData("Pose correction", "off (Constants.create has no CorrectedLocalizer)");
        } else {
            double[] offset = localizer.offset();
            telemetry.addData("Pose correction applied", "(%.1f, %.1f) in", offset[0], offset[1]);
        }
        turret.addTelemetry(telemetry);
        flywheel.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        dropdown.addTelemetry(telemetry);
        pollen.addTelemetry(telemetry);
    }

    public void stop() {
        flywheel.stop();
        intake.stop();
        tracker.close();
        pollen.close();
    }
}
