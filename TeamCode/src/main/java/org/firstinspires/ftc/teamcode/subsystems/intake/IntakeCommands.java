package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

/** Pedro Ivy commands for the intake. {@link Intake#update()} still runs every loop. */
public final class IntakeCommands {
    private IntakeCommands() {}

    /**
     * Start intaking: the roller bed runs and the transfer stays off, so balls wait in it. Runs until something
     * else takes over.
     */
    public static Command intake(Intake intake) {
        return Commands.instant(intake::intake).requiring(intake);
    }

    public static Command eject(Intake intake) {
        return Commands.instant(intake::eject).requiring(intake);
    }

    public static Command stop(Intake intake) {
        return Commands.instant(intake::stop).requiring(intake);
    }

    /**
     * Run both motors toward the shooter for {@code seconds}, then stop. The intake also stops if the command is
     * interrupted, so balls never keep feeding into the flywheels.
     */
    public static Command feedFor(Intake intake, double seconds) {
        final long[] startNanos = new long[1];
        return new CommandBuilder()
                .setStart(() -> {
                    startNanos[0] = System.nanoTime();
                    intake.feed();
                })
                .setDone(() -> (System.nanoTime() - startNanos[0]) / 1e9 >= seconds)
                .setEnd(endCondition -> intake.stop())
                .requiring(intake);
    }

    /** Shoot everything that is loaded, back to back ({@link IntakeConstants#FEED_ALL_SEC}). */
    public static Command feedAll(Intake intake) {
        return feedFor(intake, FEED_ALL_SEC);
    }

    /** Lower the dropdown and wait until it is down. */
    public static Command deployDropdown(Dropdown dropdown) {
        return Groups.sequential(
                Commands.instant(dropdown::deploy).requiring(dropdown),
                Commands.waitUntil(dropdown::isDeployed));
    }

    /** Raise the dropdown and wait until it is up. */
    public static Command stowDropdown(Dropdown dropdown) {
        return Groups.sequential(
                Commands.instant(dropdown::stow).requiring(dropdown),
                Commands.waitUntil(dropdown::isStowed));
    }
}
