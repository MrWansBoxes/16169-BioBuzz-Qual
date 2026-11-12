package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.intake.Dropdown;
import org.firstinspires.ftc.teamcode.subsystems.intake.Gate;
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
 *                 servo 0-3   turretServoA, turretServoB, cameraTilt, gate
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
    public final Gate gate;
    public final Dropdown dropdown;
    public final PollenTracker pollen;
    private final Supplier<Pose> pose;

    public Robot(HardwareMap hardwareMap, Alliance alliance, Supplier<Pose> pose) {
        this.pose = pose;
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        tracker = new HiveTracker(hardwareMap, alliance, 0);
        turret = new Turret(hardwareMap, pose, tracker);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        dropdown = new Dropdown(hardwareMap);
        pollen = new PollenTracker(hardwareMap, 0);
    }

    /** Ready to feed a ball: turret aimed and flywheel at speed. */
    public boolean readyToShoot() {
        return turret.isOnTarget() && flywheel.isReady();
    }

    /** Once per loop, after follower.update() and Scheduler.execute(). */
    public void update() {
        turret.update();
        flywheel.update();
        intake.update();
        pollen.update(pose.get());
    }

    public void addTelemetry(Telemetry telemetry) {
        turret.addTelemetry(telemetry);
        flywheel.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        gate.addTelemetry(telemetry);
        dropdown.addTelemetry(telemetry);
        pollen.addTelemetry(telemetry);
    }

    public void stop() {
        flywheel.stop();
        intake.stop();
        gate.close();
        tracker.close();
        pollen.close();
    }
}
