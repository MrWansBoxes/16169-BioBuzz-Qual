package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.subsystems.shooter.FlywheelCommands;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretCommands;

/**
 * BIOBUZZ auto, structured like Pedro's ExampleAuto (Ivy sequential routine, follower + Scheduler in loop).
 * Red and blue share this code; poses are written for RED and rotated for BLUE by {@link Alliance#fromRed}.
 *
 * Routine: drive to a shooting spot (pre-spinning the flywheel), shoot the 4 pre-loaded POLLEN at the raised
 * CELL (a TIP is 20 points in AUTO), then park in the LOADING ZONE (LEAVE 3 + PARK 5). Add POLLEN cycles
 * (GARDEN / FLOWERS) between the shot and the park once this works.
 *
 * The robot pose and the HIVE state (which CELL is raised, camera correction) are saved to {@link MatchState}
 * every loop for {@link BioBuzzTeleOp}.
 */
public abstract class BioBuzzAuto extends OpMode {
    protected abstract Alliance alliance();

    // ---------------- RED poses (inches, degrees) - placeholders, measure on the field ----------------
    // Red ROBOTS stage along the red wall (x = 0) next to the LOADING ZONE (red LOADING ZONE: x 0-11, y ~96-120).
    // Heading 0 = facing the blue wall. Robot center = half the robot length off the wall.
    public static double START_X = 9, START_Y = 84, START_HEADING = 0;
    /** Shooting spot: > ~4 ft from the raised CELL so the ball comes down into it (see turret notes). */
    public static double SHOOT_X = 30, SHOOT_Y = 110, SHOOT_HEADING = 0;
    /** Partly inside the LOADING ZONE counts as PARK. */
    public static double PARK_X = 12, PARK_Y = 108, PARK_HEADING = 0;

    public static int PRELOAD_COUNT = 4;
    /** Give up on shooting after this long so the robot still parks. */
    public static double SHOOT_TIMEOUT_MS = 8000;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Follower follower;
    private Robot robot;
    private Pose startPose, shootPose, parkPose;

    private Path startToShoot() {
        return line(startPose, shootPose).linear(startPose, shootPose);
    }

    private Path shootToPark() {
        return line(shootPose, parkPose).linear(shootPose, parkPose);
    }

    private Command autoRoutine() {
        return sequential(
                TurretCommands.track(robot.turret),
                // Spin the flywheel up for the current distance while driving.
                deadline(follow(follower, startToShoot()),
                        FlywheelCommands.spinForTarget(robot.flywheel, robot.turret)),
                race(FlywheelCommands.shoot(robot.flywheel, robot.turret,
                                IntakeCommands.feedEach(robot.intake, robot.gate, PRELOAD_COUNT, robot::readyToShoot)),
                        Commands.waitMs(SHOOT_TIMEOUT_MS)),
                FlywheelCommands.stop(robot.flywheel),
                // TODO: collect and shoot more POLLEN here once the paths are drawn in the Pedro visualizer:
                // PollenCommands.collectNearest (floor) and IntakeCommands.deployDropdown (FLOWERS).
                follow(follower, shootToPark())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        Alliance alliance = alliance();
        startPose = alliance.fromRed(poseFactory.of(START_X, START_Y, START_HEADING));
        shootPose = alliance.fromRed(poseFactory.of(SHOOT_X, SHOOT_Y, SHOOT_HEADING));
        parkPose = alliance.fromRed(poseFactory.of(PARK_X, PARK_Y, PARK_HEADING));

        follower = Constants.create(hardwareMap);
        if (follower == null) {
            throw new IllegalStateException("Pedro is not configured yet: fill in pedro/Constants.create()");
        }
        follower.setPose(startPose);
        follower.update();

        // Built in init so the turret camera is already streaming at the start.
        robot = new Robot(hardwareMap, alliance, follower::pose);
        MatchState.saveFromAuto(follower.pose(), alliance, robot.tracker);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Start pose", startPose);
        telemetry.addData("Raised CELL at start", robot.tracker.raisedCell());
        telemetry.addData("Camera", robot.tracker.cameraState());
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        robot.update();
        MatchState.saveFromAuto(follower.pose(), alliance(), robot.tracker);

        Pose pose = follower.pose();
        telemetry.addData("X / Y / heading", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Follower mode", follower.mode());
        robot.addTelemetry(telemetry);
    }

    @Override
    public void stop() {
        if (follower != null && robot != null) {
            MatchState.saveFromAuto(follower.pose(), alliance(), robot.tracker);
            robot.stop();
        }
        Scheduler.reset();
    }
}
