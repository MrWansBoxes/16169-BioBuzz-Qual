package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.subsystems.shooter.FlywheelConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Counter-rotating dual flywheel. Each wheel has its own velocity loop because the two motors/wheels never
 * match exactly, and a mismatch shows up directly as unwanted spin on the ball.
 *
 * Call {@link #update()} once per loop. Enable bulk caching (LynxModule.BulkCachingMode.AUTO) in the OpMode
 * so the two velocity reads don't each cost a separate hub transaction.
 */
public class Flywheel {
    private final DcMotorEx motorA;
    private final DcMotorEx motorB;
    private final VoltageSensor battery;

    private double targetRpm = 0;
    private double rpmA = 0, rpmB = 0;
    private double readySinceNanos = -1;

    public Flywheel(HardwareMap hardwareMap) {
        motorA = setUp(hardwareMap.get(DcMotorEx.class, MOTOR_A_NAME), MOTOR_A_REVERSED);
        motorB = setUp(hardwareMap.get(DcMotorEx.class, MOTOR_B_NAME), MOTOR_B_REVERSED);
        battery = hardwareMap.voltageSensor.iterator().next();
    }

    private static DcMotorEx setUp(DcMotorEx motor, boolean reversed) {
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        // We run our own velocity loop; the encoder is still read in this mode.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Coast down instead of braking the wheels (and the turret servos) hard.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return motor;
    }

    public void update() {
        rpmA = motorA.getVelocity() / TICKS_PER_REV * 60.0;
        rpmB = motorB.getVelocity() / TICKS_PER_REV * 60.0;

        double targetA = targetRpm * (1 + SPIN_BIAS);
        double targetB = targetRpm * (1 - SPIN_BIAS);
        double voltageScale = NOMINAL_VOLTAGE / Math.max(battery.getVoltage(), 1.0);
        motorA.setPower(power(targetA, rpmA, A_KS, A_KV, A_KP, voltageScale));
        motorB.setPower(power(targetB, rpmB, B_KS, B_KV, B_KP, voltageScale));

        boolean inTolerance = targetRpm > 0
                && Math.abs(targetA - rpmA) < READY_TOLERANCE_RPM
                && Math.abs(targetB - rpmB) < READY_TOLERANCE_RPM;
        if (!inTolerance) readySinceNanos = -1;
        else if (readySinceNanos < 0) readySinceNanos = System.nanoTime();
    }

    private static double power(double target, double measured, double kS, double kV, double kP, double voltageScale) {
        if (target <= 0) return 0;
        if (target - measured > FULL_POWER_BELOW_RPM) return 1;
        double power = kS + kV * target + kP * (target - measured);
        // Never drive backwards to slow down; let it coast. Reversing a loaded flywheel is hard on the gearbox.
        return Range.clip(power * voltageScale, 0, 1);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0, rpm);
    }

    /** Set speed for a shot from this horizontal distance (inches) using the tuning table. */
    public void setTargetForDistance(double distanceInches) {
        setTargetRpm(rpmForDistance(distanceInches));
    }

    public void idle() {
        setTargetRpm(IDLE_RPM);
    }

    public void stop() {
        setTargetRpm(0);
    }

    public boolean isReady() {
        return readySinceNanos >= 0 && (System.nanoTime() - readySinceNanos) / 1e9 >= READY_TIME_SEC;
    }

    public double targetRpm() {
        return targetRpm;
    }

    public static double rpmForDistance(double distance) {
        double[] d = TABLE_DISTANCE_IN;
        double[] r = TABLE_RPM;
        if (Double.isNaN(distance)) return r[0];
        if (distance <= d[0]) return r[0];
        for (int i = 1; i < d.length; i++) {
            if (distance <= d[i]) {
                double t = (distance - d[i - 1]) / (d[i] - d[i - 1]);
                return r[i - 1] + t * (r[i] - r[i - 1]);
            }
        }
        return r[r.length - 1];
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Flywheel target (rpm)", "%.0f  (A %.0f / B %.0f with spin bias)",
                targetRpm, targetRpm * (1 + SPIN_BIAS), targetRpm * (1 - SPIN_BIAS));
        telemetry.addData("Flywheel actual (rpm)", "A %.0f / B %.0f", rpmA, rpmB);
        telemetry.addData("Flywheel ready", isReady());
    }
}
