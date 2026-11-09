package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Gate;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.turret.HiveTracker;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretCommands;

import java.util.function.Supplier;

/**
 * Bring-up OpMode for the turret, flywheel, intake and gate.
 *
 * Init: X = blue, B = red, dpad up/down = which CELL starts raised.
 * Run:  A = track, Y = hold forward, bumpers = trim -/+ 1 degree.
 *       Hold right trigger = flywheel at table speed for the current distance.
 *       Dpad up/down = manual RPM offset +/- 50 on top of the table (to find the right RPM for each distance;
 *       record the "Distance" and "Flywheel target" telemetry into FlywheelConstants), dpad left = reset offset.
 *       Hold left trigger = intake, hold B = eject.
 *       X = shoot one ball once the flywheel is ready, dpad right = feed everything immediately.
 *
 * Until Pedro is tuned ({@link Constants#create} returns null) the robot pose is fixed at {@link #START_POSE},
 * so put the robot there and leave it still; the turret will still follow the HIVE using the camera correction.
 */
@TeleOp
public class TurretTrackingTest extends OpMode {
    /** Pedro coordinates: audience-side tile row, centered, facing the far wall (see FAR_WALL_DIRECTION_DEG). */
    public static Pose START_POSE = new Pose(72, 24, Math.toRadians(90));

    private Alliance alliance = Alliance.RED;
    private HiveTracker.Cell startRaised = HiveTracker.startingRaisedCell(Alliance.RED);
    private Follower follower;
    private Turret turret;
    private Flywheel flywheel;
    private Intake intake;
    private Gate gate;
    private Command feeding = Command.NOOP;
    private double rpmOffset = 0;
    private boolean lastLeftBumper, lastRightBumper, lastDpadUp, lastDpadDown, lastX, lastDpadRight;

    @Override
    public void init() {
        Scheduler.reset();
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        follower = Constants.create(hardwareMap);
        if (follower != null) follower.setPose(START_POSE);
    }

    @Override
    public void init_loop() {
        if (gamepad1.x && alliance != Alliance.BLUE) {
            alliance = Alliance.BLUE;
            startRaised = HiveTracker.startingRaisedCell(alliance);
        }
        if (gamepad1.b && alliance != Alliance.RED) {
            alliance = Alliance.RED;
            startRaised = HiveTracker.startingRaisedCell(alliance);
        }
        if (gamepad1.dpad_up) startRaised = HiveTracker.Cell.SCORING;
        if (gamepad1.dpad_down) startRaised = HiveTracker.Cell.AUDIENCE;
        telemetry.addData("Alliance (X blue / B red)", alliance);
        telemetry.addData("Raised CELL at start (dpad)", startRaised);
        telemetry.addData("Follower", follower == null ? "NOT CONFIGURED - using fixed START_POSE" : "ok");
    }

    @Override
    public void start() {
        // The camera needs the alliance, so the turret is built once the driver has chosen.
        HiveTracker tracker = new HiveTracker(hardwareMap, alliance, 0);
        tracker.setRaisedCell(startRaised);
        Supplier<Pose> pose = follower != null ? follower::pose : () -> START_POSE;
        turret = new Turret(hardwareMap, pose, tracker);
        TurretCommands.track(turret).schedule();
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            follower.update();
        }

        if (gamepad1.a) TurretCommands.track(turret).schedule();
        if (gamepad1.y) TurretCommands.holdAngle(turret, 0).schedule();
        if (gamepad1.left_bumper && !lastLeftBumper) turret.setTrimDegrees(turret.trimDegrees() - 1);
        if (gamepad1.right_bumper && !lastRightBumper) turret.setTrimDegrees(turret.trimDegrees() + 1);
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;

        if (gamepad1.dpad_up && !lastDpadUp) rpmOffset += 50;
        if (gamepad1.dpad_down && !lastDpadDown) rpmOffset -= 50;
        if (gamepad1.dpad_left) rpmOffset = 0;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        if (gamepad1.x && !lastX) {
            feeding = IntakeCommands.feedEach(intake, gate, 1, flywheel::isReady);
            feeding.schedule();
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            feeding = IntakeCommands.feedAll(intake, gate);
            feeding.schedule();
        }
        lastX = gamepad1.x;
        lastDpadRight = gamepad1.dpad_right;
        if (!feeding.isScheduled()) {
            if (gamepad1.left_trigger > 0.5) IntakeCommands.intake(intake, gate).schedule();
            else if (gamepad1.b) IntakeCommands.eject(intake).schedule();
            else if (intake.mode() != Intake.Mode.STOP) IntakeCommands.stop(intake).schedule();
        }

        Scheduler.execute();
        turret.update();
        if (gamepad1.right_trigger > 0.5) {
            flywheel.setTargetRpm(Flywheel.rpmForDistance(turret.targetDistance()) + rpmOffset);
        } else {
            flywheel.stop();
        }
        flywheel.update();
        intake.update();

        if (follower != null) telemetry.addData("Pose", follower.pose());
        telemetry.addData("Distance (in) / RPM offset", "%.1f / %+.0f", turret.targetDistance(), rpmOffset);
        turret.addTelemetry(telemetry);
        flywheel.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        gate.addTelemetry(telemetry);
    }

    @Override
    public void stop() {
        if (flywheel != null) flywheel.stop();
        if (intake != null) intake.stop();
        if (gate != null) gate.close();
        if (turret != null) turret.tracker().close();
        Scheduler.reset();
    }
}
