package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

/** Pedro Ivy commands for the turret. {@link Turret#update()} still runs every loop; these only change its mode. */
public final class TurretCommands {
    private TurretCommands() {}

    /** Start tracking the raised CELL. Tracking continues after this command ends. */
    public static Command track(Turret turret) {
        return Commands.instant(turret::track).requiring(turret);
    }

    /** Point at a fixed angle relative to the robot (0 = forward), e.g. to stow for intaking. */
    public static Command holdAngle(Turret turret, double degrees) {
        return Commands.instant(() -> turret.holdAngle(degrees)).requiring(turret);
    }

    public static Command waitUntilOnTarget(Turret turret) {
        return Commands.waitUntil(turret::isOnTarget);
    }

    /** Track, wait until aimed, then run {@code shoot} (e.g. open the gate and feed). */
    public static Command aimThen(Turret turret, Command shoot) {
        return Groups.sequential(track(turret), waitUntilOnTarget(turret), shoot);
    }
}
