package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.AxonServo;

/**
 * The dropdown that reaches POLLEN on the FLOWERS, lowered and raised by two Axon Minis winding string spools.
 * Like {@link Gate}, there is no feedback, so "down"/"up" are estimated from
 * {@link IntakeConstants#DROPDOWN_MOVE_SEC}.
 * Stows when constructed so the robot starts inside its starting size.
 */
public class Dropdown {
    private final AxonServo left;
    private final AxonServo right;
    private boolean deployed = false;
    private long commandedNanos = 0;

    public Dropdown(HardwareMap hardwareMap) {
        left = new AxonServo(hardwareMap, DROPDOWN_LEFT_NAME, "");
        right = new AxonServo(hardwareMap, DROPDOWN_RIGHT_NAME, "");
        stow();
    }

    public void deploy() {
        set(true);
    }

    public void stow() {
        set(false);
    }

    public void toggle() {
        set(!deployed);
    }

    private void set(boolean wantDeployed) {
        if (wantDeployed != deployed || commandedNanos == 0) commandedNanos = System.nanoTime();
        deployed = wantDeployed;
        left.setPosition(wantDeployed ? DROPDOWN_LEFT_DEPLOYED : DROPDOWN_LEFT_STOWED);
        right.setPosition(wantDeployed ? DROPDOWN_RIGHT_DEPLOYED : DROPDOWN_RIGHT_STOWED);
    }

    private boolean settled() {
        return (System.nanoTime() - commandedNanos) / 1e9 >= DROPDOWN_MOVE_SEC;
    }

    /** True once the dropdown has had time to fully lower. */
    public boolean isDeployed() {
        return deployed && settled();
    }

    /** True once the dropdown has had time to fully raise. */
    public boolean isStowed() {
        return !deployed && settled();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Dropdown",
                deployed ? (isDeployed() ? "DOWN" : "lowering") : (isStowed() ? "STOWED" : "raising"));
    }
}
