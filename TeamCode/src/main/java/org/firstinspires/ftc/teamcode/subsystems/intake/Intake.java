package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Two-motor intake: A = front roller bed, B = transfer pair at the back. Modes set the power of both motors from
 * {@link IntakeConstants}; call {@link #update()} once per loop. While intaking, a stalled roller bed (optional
 * current check) is reversed briefly to clear it.
 */
public class Intake {
    public enum Mode { STOP, INTAKE, FEED, EJECT }

    private final DcMotorEx motorA;
    private final DcMotorEx motorB;

    private Mode mode = Mode.STOP;
    private long overCurrentSinceNanos = -1;
    private long unjamUntilNanos = 0;
    private double currentA = 0;

    public Intake(HardwareMap hardwareMap) {
        motorA = setUp(hardwareMap.get(DcMotorEx.class, MOTOR_A_NAME), MOTOR_A_REVERSED);
        motorB = setUp(hardwareMap.get(DcMotorEx.class, MOTOR_B_NAME), MOTOR_B_REVERSED);
    }

    private static DcMotorEx setUp(DcMotorEx motor, boolean reversed) {
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Brake so stored balls don't roll back out when the intake stops.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public void update() {
        long now = System.nanoTime();
        double powerA, powerB;
        switch (mode) {
            case INTAKE:
                powerA = INTAKE_POWER_A;
                powerB = INTAKE_POWER_B;
                if (now < unjamUntilNanos) {
                    powerA = EJECT_POWER_A;
                } else if (jammed(now)) {
                    unjamUntilNanos = now + (long) (UNJAM_TIME_SEC * 1e9);
                    overCurrentSinceNanos = -1;
                    powerA = EJECT_POWER_A;
                }
                break;
            case FEED:
                powerA = FEED_POWER_A;
                powerB = FEED_POWER_B;
                break;
            case EJECT:
                powerA = EJECT_POWER_A;
                powerB = EJECT_POWER_B;
                break;
            default:
                powerA = 0;
                powerB = 0;
        }
        motorA.setPower(powerA);
        motorB.setPower(powerB);
    }

    private boolean jammed(long now) {
        if (JAM_CURRENT_AMPS <= 0) return false;
        currentA = motorA.getCurrent(CurrentUnit.AMPS);
        if (currentA < JAM_CURRENT_AMPS) {
            overCurrentSinceNanos = -1;
            return false;
        }
        if (overCurrentSinceNanos < 0) overCurrentSinceNanos = now;
        return (now - overCurrentSinceNanos) / 1e9 >= JAM_TIME_SEC;
    }

    public void intake() {
        if (mode != Mode.INTAKE) {
            overCurrentSinceNanos = -1;
            unjamUntilNanos = 0;
        }
        mode = Mode.INTAKE;
    }

    public void feed() {
        mode = Mode.FEED;
    }

    public void eject() {
        mode = Mode.EJECT;
    }

    public void stop() {
        mode = Mode.STOP;
    }

    public Mode mode() {
        return mode;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", JAM_CURRENT_AMPS > 0 ? String.format("%s  (A %.1f A)", mode, currentA) : mode);
    }
}
