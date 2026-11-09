package org.firstinspires.ftc.teamcode.subsystems.pollen;

import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.subsystems.pollen.PollenConstants.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.intake.Gate;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCommands;

/** Pedro Ivy commands that use the POLLEN camera. */
public final class PollenCommands {
    private PollenCommands() {}

    /**
     * Drive straight at the nearest POLLEN the camera sees right now, facing it, with the intake running, until
     * the ball should be inside the roller bed ({@link PollenConstants#PICKUP_BALL_FORWARD}) or
     * {@link PollenConstants#PICKUP_TIMEOUT_MS} runs out. Does nothing if no POLLEN is visible.
     * The intake keeps running afterwards; stop it or start shooting next.
     */
    public static Command collectNearest(Follower follower, PollenTracker pollen, Intake intake, Gate gate) {
        return Commands.lazy(() -> {
            PollenTracker.Sighting target = pollen.nearest();
            if (target == null) return Command.NOOP;

            Pose start = follower.pose();
            double heading = Math.atan2(target.fieldY - start.y(), target.fieldX - start.x());
            Pose end = new Pose(
                    target.fieldX - PICKUP_BALL_FORWARD * Math.cos(heading),
                    target.fieldY - PICKUP_BALL_FORWARD * Math.sin(heading),
                    heading);
            return Groups.race(
                    Groups.sequential(
                            IntakeCommands.intake(intake, gate),
                            PedroCommands.follow(follower, line(start, end).constant(heading))),
                    Commands.waitMs(PICKUP_TIMEOUT_MS));
        });
    }
}
