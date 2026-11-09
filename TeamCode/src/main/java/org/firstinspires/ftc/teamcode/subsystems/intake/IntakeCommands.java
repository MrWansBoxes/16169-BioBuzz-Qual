package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import java.util.function.BooleanSupplier;

/** Pedro Ivy commands for the intake and gate. {@link Intake#update()} still runs every loop. */
public final class IntakeCommands {
    private IntakeCommands() {}

    /** Close the gate and start intaking; balls collect behind the gate. Runs until something else takes over. */
    public static Command intake(Intake intake, Gate gate) {
        return Commands.instant(() -> {
            gate.close();
            intake.intake();
        }).requiring(intake, gate);
    }

    public static Command eject(Intake intake) {
        return Commands.instant(intake::eject).requiring(intake);
    }

    public static Command stop(Intake intake) {
        return Commands.instant(intake::stop).requiring(intake);
    }

    /**
     * Open the gate, wait for it to open, push balls for {@code seconds}, then stop and close the gate.
     * The gate also closes if the command is interrupted, so balls never leak into the flywheels.
     */
    public static Command feedFor(Intake intake, Gate gate, double seconds) {
        final long[] startNanos = new long[1];
        return new CommandBuilder()
                .setStart(() -> {
                    startNanos[0] = System.nanoTime();
                    gate.open();
                    intake.stop();
                })
                .setExecute(() -> {
                    if (gate.isOpen()) intake.feed();
                })
                .setDone(() -> (System.nanoTime() - startNanos[0]) / 1e9 >= GATE_MOVE_SEC + seconds)
                .setEnd(endCondition -> {
                    intake.stop();
                    gate.close();
                })
                .requiring(intake, gate);
    }

    public static Command feedOne(Intake intake, Gate gate) {
        return feedFor(intake, gate, ONE_BALL_FEED_SEC);
    }

    public static Command feedAll(Intake intake, Gate gate) {
        return feedFor(intake, gate, FEED_ALL_SEC);
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

    /**
     * Shoot {@code count} balls one at a time, waiting for {@code ready} (flywheel back up to speed and turret
     * on target) before each one. Slower than {@link #feedAll} but every ball leaves at the right speed.
     */
    public static Command feedEach(Intake intake, Gate gate, int count, BooleanSupplier ready) {
        return Groups.repeat(
                Groups.sequential(
                        Commands.waitUntil(() -> ready.getAsBoolean() && gate.isClosed()),
                        feedOne(intake, gate)),
                count);
    }
}
