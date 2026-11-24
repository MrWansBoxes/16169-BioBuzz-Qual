package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Locale;

/**
 * Two-motor intake: A = front roller bed, B = transfer pair at the back. Modes set the speed of both motors from
 * {@link IntakeConstants}, held by the hub's velocity loop on each motor's encoder; call {@link #update()} once per
 * loop. There is no gate: while intaking the transfer (B) stays stopped and holds the balls, and FEED runs both
 * motors to shoot (B launches the balls up into the flywheels). While intaking, a stalled roller bed (optional
 * current check) is reversed briefly to clear it.
 *
 * If a motor's encoder reads (almost) nothing, or counts the wrong way, that motor falls back to plain power
 * instead of letting the velocity loop run it flat out; telemetry then shows "NO ENCODER".
 */
public class Intake {
    public enum Mode { STOP, INTAKE, FEED, EJECT }

    /** One motor on the hub's velocity loop, with the encoder check. */
    private static final class VelocityMotor {
        final DcMotorEx motor;
        final String name;
        double targetRpm;
        long runningSinceNanos = -1;
        boolean encoderFailed;

        VelocityMotor(HardwareMap hardwareMap, String name, boolean reversed) {
            this.name = name;
            motor = hardwareMap.get(DcMotorEx.class, name);
            motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Brake so stored balls don't roll back out (or forward into the flywheels) when stopped.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }

        void run(double rpm, long now) {
            // Restart the encoder check whenever the target changes direction or starts from a stop.
            if (rpm == 0 || Math.signum(rpm) != Math.signum(targetRpm)) runningSinceNanos = rpm == 0 ? -1 : now;
            targetRpm = rpm;

            if (encoderFailed) {
                motor.setPower(Math.max(-1, Math.min(1, rpm / OPEN_LOOP_FULL_RPM)));
                return;
            }
            if (rpm == 0) {
                motor.setPower(0);
                return;
            }
            motor.setVelocity(rpm * TICKS_PER_REV / 60.0);
            if ((now - runningSinceNanos) / 1e9 >= ENCODER_CHECK_SEC
                    && measuredRpm() * Math.signum(rpm) < ENCODER_FAIL_FRACTION * Math.abs(rpm)) {
                encoderFailed = true;
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(Math.max(-1, Math.min(1, rpm / OPEN_LOOP_FULL_RPM)));
            }
        }

        double measuredRpm() {
            return motor.getVelocity() * 60.0 / TICKS_PER_REV;
        }

        String status() {
            if (encoderFailed) {
                return String.format(Locale.US, "NO ENCODER (plain power) target %.0f", targetRpm);
            }
            return String.format(Locale.US, "%.0f / %.0f rpm", targetRpm, measuredRpm());
        }
    }

    private final VelocityMotor motorA;
    private final VelocityMotor motorB;

    private Mode mode = Mode.STOP;
    private long overCurrentSinceNanos = -1;
    private long unjamUntilNanos = 0;
    private double currentA = 0;

    public Intake(HardwareMap hardwareMap) {
        motorA = new VelocityMotor(hardwareMap, MOTOR_A_NAME, MOTOR_A_REVERSED);
        motorB = new VelocityMotor(hardwareMap, MOTOR_B_NAME, MOTOR_B_REVERSED);
    }

    public void update() {
        long now = System.nanoTime();
        double rpmA, rpmB;
        switch (mode) {
            case INTAKE:
                rpmA = INTAKE_RPM_A;
                rpmB = INTAKE_RPM_B;
                if (now < unjamUntilNanos) {
                    rpmA = EJECT_RPM_A;
                } else if (jammed(now)) {
                    unjamUntilNanos = now + (long) (UNJAM_TIME_SEC * 1e9);
                    overCurrentSinceNanos = -1;
                    rpmA = EJECT_RPM_A;
                }
                break;
            case FEED:
                rpmA = FEED_RPM_A;
                rpmB = FEED_RPM_B;
                break;
            case EJECT:
                rpmA = EJECT_RPM_A;
                rpmB = EJECT_RPM_B;
                break;
            default:
                rpmA = 0;
                rpmB = 0;
        }
        motorA.run(rpmA, now);
        motorB.run(rpmB, now);
    }

    private boolean jammed(long now) {
        if (JAM_CURRENT_AMPS <= 0) return false;
        currentA = motorA.motor.getCurrent(CurrentUnit.AMPS);
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

    /** True if either motor lost its encoder and is running on plain power. */
    public boolean encoderFault() {
        return motorA.encoderFailed || motorB.encoderFailed;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", JAM_CURRENT_AMPS > 0 ? String.format(Locale.US, "%s  (A %.1f A)", mode, currentA)
                : mode);
        telemetry.addData("Intake A / B target / measured", "%s  |  %s", motorA.status(), motorB.status());
    }
}
