package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretCommands;

/** Pedro Ivy commands for the flywheel. {@link Flywheel#update()} still runs every loop. */
public final class FlywheelCommands {
    private FlywheelCommands() {}

    /** Keep the flywheel speed matched to the turret's current distance to the raised CELL until cancelled. */
    public static Command spinForTarget(Flywheel flywheel, Turret turret) {
        return Commands.infinite(() -> flywheel.setTargetForDistance(turret.targetDistance()))
                .requiring(flywheel);
    }

    public static Command spinAt(Flywheel flywheel, double rpm) {
        return Commands.instant(() -> flywheel.setTargetRpm(rpm)).requiring(flywheel);
    }

    public static Command stop(Flywheel flywheel) {
        return Commands.instant(flywheel::stop).requiring(flywheel);
    }

    public static Command waitUntilReady(Flywheel flywheel) {
        return Commands.waitUntil(flywheel::isReady);
    }

    /**
     * Aim and spin up together, wait until both the turret and the flywheel are ready, then run {@code feed}
     * (e.g. open the gate). The flywheel keeps tracking distance while feeding and idles afterwards.
     */
    public static Command shoot(Flywheel flywheel, Turret turret, Command feed) {
        return Groups.deadline(
                Groups.sequential(
                        TurretCommands.track(turret),
                        Commands.waitUntil(() -> turret.isOnTarget() && flywheel.isReady()),
                        feed),
                spinForTarget(flywheel, turret)
        ).then(Commands.instant(flywheel::idle).requiring(flywheel));
    }
}
