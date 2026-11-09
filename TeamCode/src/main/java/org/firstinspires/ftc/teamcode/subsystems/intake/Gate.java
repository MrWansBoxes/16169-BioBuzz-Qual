package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.AxonServo;

/**
 * Servo gate between the intake and the flywheels. A servo gives no feedback, so "open"/"closed" are
 * estimated from how long ago the position was commanded ({@link IntakeConstants#GATE_MOVE_SEC}).
 */
public class Gate {
    private final AxonServo servo;
    private boolean open = false;
    private long commandedNanos = 0;

    public Gate(HardwareMap hardwareMap) {
        servo = new AxonServo(hardwareMap, GATE_SERVO_NAME, "");
        close();
    }

    public void open() {
        set(true);
    }

    public void close() {
        set(false);
    }

    private void set(boolean wantOpen) {
        if (wantOpen != open || commandedNanos == 0) commandedNanos = System.nanoTime();
        open = wantOpen;
        servo.setPosition(wantOpen ? GATE_OPEN : GATE_CLOSED);
    }

    private boolean settled() {
        return (System.nanoTime() - commandedNanos) / 1e9 >= GATE_MOVE_SEC;
    }

    /** True once the gate has had time to fully open. */
    public boolean isOpen() {
        return open && settled();
    }

    /** True once the gate has had time to fully close. */
    public boolean isClosed() {
        return !open && settled();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Gate", open ? (isOpen() ? "OPEN" : "opening") : (isClosed() ? "CLOSED" : "closing"));
    }
}
