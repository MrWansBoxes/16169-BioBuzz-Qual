package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.subsystems.pollen.PollenConstants;
import org.firstinspires.ftc.teamcode.subsystems.pollen.PollenTracker;
import org.firstinspires.ftc.teamcode.subsystems.shooter.FlywheelCommands;
import org.firstinspires.ftc.teamcode.subsystems.turret.HiveTracker;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretCommands;

/**
 * Match TeleOp. If an auto just ran, it continues from auto's final pose and HIVE state ({@link MatchState});
 * otherwise it starts from the auto start pose for the alliance chosen in init.
 *
 * gamepad1 (driver): sticks = field-centric drive, options = re-zero field-centric (point the robot away from
 *   you first), hold right trigger = POLLEN assist (turns toward the nearest POLLEN the front camera sees and
 *   runs the intake; the left stick still drives). In init: X = blue, B = red, back = ignore auto's saved state.
 * gamepad2 (operator): left trigger = intake, B = eject, hold right trigger = pre-spin flywheel,
 *   right bumper = shoot everything (one ball at a time), left bumper = shoot one, X = cancel shooting,
 *   A = turret track, Y = turret forward, dpad left/right = turret trim, dpad up/down = override raised CELL
 *   (up = SCORING / far side, down = AUDIENCE side), left stick button = lower / raise the dropdown.
 */
@TeleOp(name = BioBuzzTeleOp.NAME)
public class BioBuzzTeleOp extends OpMode {
    public static final String NAME = "BioBuzz TeleOp";
    public static int BALLS_PER_VOLLEY = 4;

    private Follower follower;
    private Robot robot;
    private Alliance alliance = Alliance.RED;
    private boolean useAutoState;
    private double driverForwardHeading;
    private Command shooting = Command.NOOP;
    private boolean pollenAssist;
    private boolean lastRightBumper, lastLeftBumper, lastDpadLeft, lastDpadRight, lastLeftStick;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        if (follower == null) {
            throw new IllegalStateException("Pedro is not configured yet: fill in pedro/Constants.create()");
        }
        useAutoState = MatchState.hasFreshAutoState();
        if (useAutoState) alliance = MatchState.alliance;
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;
        if (gamepad1.back) useAutoState = false;

        telemetry.addData("Alliance (X blue / B red)", alliance);
        if (useAutoState) {
            telemetry.addData("Using auto state", "pose %s, raised %s, %.0f s old (back = ignore)",
                    MatchState.pose, MatchState.raisedCell, MatchState.ageSeconds());
        } else {
            telemetry.addData("Using auto state", "no - starting from the auto start pose");
        }
    }

    @Override
    public void start() {
        Pose startPose = useAutoState
                ? MatchState.pose
                : alliance.fromRed(new Pose(BioBuzzAuto.START_X, BioBuzzAuto.START_Y,
                        Math.toRadians(BioBuzzAuto.START_HEADING)));
        follower.setPose(startPose);
        follower.update();

        robot = new Robot(hardwareMap, alliance, follower::pose);
        if (useAutoState && MatchState.raisedCell != null) {
            robot.tracker.restore(MatchState.raisedCell, MatchState.hiveCorrectionX, MatchState.hiveCorrectionY,
                    MatchState.hiveSeenByCamera);
        }
        MatchState.clear();

        // Drivers stand at their alliance wall: red looks toward +X (heading 0), blue toward -X.
        driverForwardHeading = alliance == Alliance.RED ? 0 : Math.PI;
        TurretCommands.track(robot.turret).schedule();
    }

    @Override
    public void loop() {
        drive();
        operate();

        Scheduler.execute();
        robot.update();

        Pose pose = follower.pose();
        telemetry.addData("X / Y / heading", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        robot.addTelemetry(telemetry);
    }

    private void drive() {
        if (gamepad1.options) driverForwardHeading = follower.pose().heading();
        pollenAssist = gamepad1.right_trigger > 0.5;
        double turn = gamepad1.right_stick_x;
        PollenTracker.Sighting pollen = robot.pollen.nearest();
        if (pollenAssist && pollen != null) {
            // Turn is positive to the right (like the stick); bearing is positive to the left.
            turn = -Range.clip(PollenConstants.ASSIST_KP * pollen.bearing,
                    -PollenConstants.ASSIST_MAX_TURN, PollenConstants.ASSIST_MAX_TURN);
        }
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                turn,
                follower.pose().heading() - driverForwardHeading);
        follower.manual(powers);
        follower.update();
    }

    private void operate() {
        // Turret
        if (gamepad2.a) TurretCommands.track(robot.turret).schedule();
        if (gamepad2.y) TurretCommands.holdAngle(robot.turret, 0).schedule();
        if (gamepad2.dpad_left && !lastDpadLeft) robot.turret.setTrimDegrees(robot.turret.trimDegrees() + 1);
        if (gamepad2.dpad_right && !lastDpadRight) robot.turret.setTrimDegrees(robot.turret.trimDegrees() - 1);
        if (gamepad2.dpad_up) robot.tracker.setRaisedCell(HiveTracker.Cell.SCORING);
        if (gamepad2.dpad_down) robot.tracker.setRaisedCell(HiveTracker.Cell.AUDIENCE);

        // Dropdown
        if (gamepad2.left_stick_button && !lastLeftStick) robot.dropdown.toggle();

        // Shooting
        if (gamepad2.right_bumper && !lastRightBumper) startShooting(BALLS_PER_VOLLEY);
        if (gamepad2.left_bumper && !lastLeftBumper) startShooting(1);
        if (gamepad2.x) shooting.cancel();

        if (!shooting.isScheduled()) {
            if (gamepad2.right_trigger > 0.5) {
                robot.flywheel.setTargetForDistance(robot.turret.targetDistance());
            } else {
                robot.flywheel.idle();
            }
            if (gamepad2.left_trigger > 0.5 || pollenAssist) IntakeCommands.intake(robot.intake, robot.gate).schedule();
            else if (gamepad2.b) IntakeCommands.eject(robot.intake).schedule();
            else if (robot.intake.mode() != Intake.Mode.STOP) IntakeCommands.stop(robot.intake).schedule();
        }

        lastRightBumper = gamepad2.right_bumper;
        lastLeftBumper = gamepad2.left_bumper;
        lastDpadLeft = gamepad2.dpad_left;
        lastDpadRight = gamepad2.dpad_right;
        lastLeftStick = gamepad2.left_stick_button;
    }

    private void startShooting(int balls) {
        shooting.cancel();
        shooting = FlywheelCommands.shoot(robot.flywheel, robot.turret,
                IntakeCommands.feedEach(robot.intake, robot.gate, balls, robot::readyToShoot));
        shooting.schedule();
    }

    @Override
    public void stop() {
        if (robot != null) robot.stop();
        Scheduler.reset();
    }
}
