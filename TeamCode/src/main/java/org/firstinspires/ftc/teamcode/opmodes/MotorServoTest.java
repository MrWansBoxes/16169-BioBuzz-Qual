package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.AxonServo;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * Moves every motor and servo by hand, one at a time. Needs no encoders and no servo feedback wires, and a device
 * missing from the robot configuration is reported instead of crashing. Put the robot on a stand first.
 *
 * Gamepad 1: dpad up/down = pick a device, A/B (hold) = move it, dpad left/right = motor power.
 *
 * Motors only run while A or B is held, and power goes back to {@link #START_POWER} on every new selection.
 * A servo stays unpowered until the first press, which sends it to a starting position; holding A or B then moves
 * it. Picking another device cuts power to it, so do that if a mechanism is grinding. The telemetry position is
 * the number to copy into the constants files.
 *
 * Servos that drive the same part are tested together and also alone, because moving them together would hide a
 * dead or swapped servo. The alone tests use a short range so the part can't be forced far by one side.
 */
@TeleOp(name = "Motor Servo Test", group = "Test")
public class MotorServoTest extends OpMode {
    public static double START_POWER = 0.3;
    public static double POWER_STEP = 0.1;
    /** Time to cross a servo's whole range while holding A or B. */
    public static double SWEEP_SEC = 3.0;
    /** Turret travel either side of center when one turret servo is tested alone. */
    public static double SOLO_TURRET_DEG = 15.0;
    /** Share of the dropdown's travel when one spool servo is tested alone. */
    public static double SOLO_DROPDOWN_FRACTION = 0.25;

    private final List<TestItem> items = new ArrayList<>();
    private final Set<String> missing = new LinkedHashSet<>();
    private int selected = 0;
    private double power = START_POWER;
    private long lastNanos = 0;

    @Override
    public void init() {
        motor(Constants.drivetrainConfig.frontLeftName.get(), Constants.drivetrainConfig.frontLeftDirection.get(),
                "A = robot forward, B = back");
        motor(Constants.drivetrainConfig.backLeftName.get(), Constants.drivetrainConfig.backLeftDirection.get(),
                "A = robot forward, B = back");
        motor(Constants.drivetrainConfig.frontRightName.get(), Constants.drivetrainConfig.frontRightDirection.get(),
                "A = robot forward, B = back");
        motor(Constants.drivetrainConfig.backRightName.get(), Constants.drivetrainConfig.backRightDirection.get(),
                "A = robot forward, B = back");
        motor(IntakeConstants.MOTOR_A_NAME, direction(IntakeConstants.MOTOR_A_REVERSED),
                "A = toward shooter, B = eject");
        motor(IntakeConstants.MOTOR_B_NAME, direction(IntakeConstants.MOTOR_B_REVERSED),
                "A = toward shooter, B = eject");
        motor(FlywheelConstants.MOTOR_A_NAME, direction(FlywheelConstants.MOTOR_A_REVERSED),
                "A = shoot direction, B = reverse");
        motor(FlywheelConstants.MOTOR_B_NAME, direction(FlywheelConstants.MOTOR_B_REVERSED),
                "A = shoot direction, B = reverse");

        servos(IntakeConstants.GATE_SERVO_NAME, "A = open, B = close", 0,
                move(IntakeConstants.GATE_SERVO_NAME, IntakeConstants.GATE_CLOSED, IntakeConstants.GATE_OPEN));

        double tiltStart = (TurretConstants.DEFAULT_PITCH_DEG - TurretConstants.MIN_PITCH_DEG)
                / (TurretConstants.MAX_PITCH_DEG - TurretConstants.MIN_PITCH_DEG);
        servos(TurretConstants.TILT_SERVO_NAME, "A = up, B = down", tiltStart,
                move(TurretConstants.TILT_SERVO_NAME, tiltPosition(TurretConstants.MIN_PITCH_DEG),
                        tiltPosition(TurretConstants.MAX_PITCH_DEG)));

        servos(IntakeConstants.DROPDOWN_LEFT_NAME + " alone", "A = lower, B = stow", 0,
                dropdownLeft(SOLO_DROPDOWN_FRACTION));
        servos(IntakeConstants.DROPDOWN_RIGHT_NAME + " alone", "A = lower, B = stow", 0,
                dropdownRight(SOLO_DROPDOWN_FRACTION));
        servos("dropdown (both)", "A = lower, B = stow", 0, dropdownLeft(1), dropdownRight(1));

        servos(TurretConstants.SERVO_A_NAME + " alone", "A = left, B = right", 0.5, turretA(SOLO_TURRET_DEG));
        servos(TurretConstants.SERVO_B_NAME + " alone", "A = left, B = right", 0.5, turretB(SOLO_TURRET_DEG));
        servos("turret (both)", "A = left, B = right", 0.5,
                turretA(TurretConstants.MAX_ANGLE_DEG), turretB(TurretConstants.MAX_ANGLE_DEG));

        for (TestItem item : items) item.off();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Put the robot on a stand, then press start.");
        telemetry.addData("Devices to test", items.size());
        telemetry.addData("Not in robot configuration", missing.isEmpty() ? "none" : missing.toString());
    }

    @Override
    public void start() {
        lastNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;

        if (gamepad1.dpadDownWasPressed()) select(selected + 1);
        if (gamepad1.dpadUpWasPressed()) select(selected - 1);
        if (gamepad1.dpadRightWasPressed()) power = Range.clip(power + POWER_STEP, POWER_STEP, 1.0);
        if (gamepad1.dpadLeftWasPressed()) power = Range.clip(power - POWER_STEP, POWER_STEP, 1.0);

        int direction = (gamepad1.a ? 1 : 0) - (gamepad1.b ? 1 : 0);
        TestItem item = items.get(selected);
        item.run(direction, dt);

        telemetry.addData("Device", "%s  (%d of %d)", item.label, selected + 1, items.size());
        telemetry.addData("Controls", item.found() ? item.hint : "NOT IN ROBOT CONFIGURATION");
        telemetry.addData("Status", item.status());
        if (item instanceof MotorItem) telemetry.addData("Power (dpad left/right)", "%.1f", power);
        telemetry.addLine("Dpad up/down = pick a device");
        if (!missing.isEmpty()) telemetry.addData("Not in robot configuration", missing.toString());
    }

    @Override
    public void stop() {
        for (TestItem item : items) item.off();
    }

    private void select(int index) {
        items.get(selected).off();
        selected = Math.floorMod(index, items.size());
        power = START_POWER;
    }

    private static DcMotorSimple.Direction direction(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    private static double tiltPosition(double pitchDeg) {
        return TurretConstants.TILT_SERVO_LEVEL + pitchDeg / (TurretConstants.TILT_DIRECTION * AxonServo.RANGE_DEG);
    }

    private ServoMove dropdownLeft(double fraction) {
        return move(IntakeConstants.DROPDOWN_LEFT_NAME, IntakeConstants.DROPDOWN_LEFT_STOWED,
                IntakeConstants.DROPDOWN_LEFT_STOWED
                        + fraction * (IntakeConstants.DROPDOWN_LEFT_DEPLOYED - IntakeConstants.DROPDOWN_LEFT_STOWED));
    }

    private ServoMove dropdownRight(double fraction) {
        return move(IntakeConstants.DROPDOWN_RIGHT_NAME, IntakeConstants.DROPDOWN_RIGHT_STOWED,
                IntakeConstants.DROPDOWN_RIGHT_STOWED
                        + fraction * (IntakeConstants.DROPDOWN_RIGHT_DEPLOYED - IntakeConstants.DROPDOWN_RIGHT_STOWED));
    }

    private ServoMove turretA(double degrees) {
        return turret(TurretConstants.SERVO_A_NAME, TurretConstants.SERVO_A_ZERO, TurretConstants.SERVO_A_DIRECTION,
                degrees);
    }

    private ServoMove turretB(double degrees) {
        return turret(TurretConstants.SERVO_B_NAME, TurretConstants.SERVO_B_ZERO, TurretConstants.SERVO_B_DIRECTION,
                degrees);
    }

    /** From this many degrees right of center to the same amount left of it, as in {@code Turret.writeServos}. */
    private ServoMove turret(String name, double zero, double servoDirection, double degrees) {
        double positionPerDegree = TurretConstants.TURRET_GEAR_RATIO / (servoDirection * AxonServo.RANGE_DEG);
        return move(name, zero - degrees * positionPerDegree, zero + degrees * positionPerDegree);
    }

    private void motor(String name, DcMotorSimple.Direction direction, String hint) {
        DcMotor motor = hardwareMap.tryGet(DcMotor.class, name);
        if (motor == null) {
            missing.add(name);
        } else {
            motor.setDirection(direction);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        items.add(new MotorItem(name, hint, motor));
    }

    private void servos(String label, String hint, double startFraction, ServoMove... moves) {
        items.add(new ServoItem(label, hint, startFraction, moves));
    }

    private ServoMove move(String name, double from, double to) {
        ServoImplEx servo = hardwareMap.tryGet(ServoImplEx.class, name);
        if (servo == null) {
            missing.add(name);
        } else {
            servo.setPwmRange(new PwmControl.PwmRange(AxonServo.PWM_MIN_US, AxonServo.PWM_MAX_US));
        }
        return new ServoMove(servo, from, to);
    }

    /** One servo's positions at the two ends of a test. */
    private static class ServoMove {
        final ServoImplEx servo;
        final double from;
        final double to;

        ServoMove(ServoImplEx servo, double from, double to) {
            this.servo = servo;
            this.from = from;
            this.to = to;
        }

        double position(double fraction) {
            return Range.clip(from + fraction * (to - from), 0, 1);
        }
    }

    private abstract static class TestItem {
        final String label;
        final String hint;

        TestItem(String label, String hint) {
            this.label = label;
            this.hint = hint;
        }

        abstract boolean found();

        /** @param direction +1 while A is held, -1 while B is held, 0 otherwise. */
        abstract void run(int direction, double dt);

        abstract void off();

        abstract String status();
    }

    private class MotorItem extends TestItem {
        private final DcMotor motor;
        private double applied = 0;

        MotorItem(String label, String hint, DcMotor motor) {
            super(label, hint);
            this.motor = motor;
        }

        @Override
        boolean found() {
            return motor != null;
        }

        @Override
        void run(int direction, double dt) {
            if (motor == null) return;
            applied = direction * power;
            motor.setPower(applied);
        }

        @Override
        void off() {
            applied = 0;
            if (motor != null) motor.setPower(0);
        }

        @Override
        String status() {
            return String.format("power %+.1f", applied);
        }
    }

    private static class ServoItem extends TestItem {
        private final double startFraction;
        private final ServoMove[] moves;
        private double fraction;
        private boolean powered = false;

        ServoItem(String label, String hint, double startFraction, ServoMove[] moves) {
            super(label, hint);
            this.startFraction = startFraction;
            this.moves = moves;
            this.fraction = startFraction;
        }

        @Override
        boolean found() {
            for (ServoMove move : moves) {
                if (move.servo == null) return false;
            }
            return true;
        }

        @Override
        void run(int direction, double dt) {
            if (direction == 0 || !found()) return;
            if (powered) fraction = Range.clip(fraction + direction * dt / SWEEP_SEC, 0, 1);
            powered = true;
            for (ServoMove move : moves) move.servo.setPosition(move.position(fraction));
        }

        @Override
        void off() {
            powered = false;
            fraction = startFraction;
            for (ServoMove move : moves) {
                if (move.servo != null) move.servo.setPwmDisable();
            }
        }

        @Override
        String status() {
            if (!powered) return "unpowered, first press moves it to its start";
            StringBuilder positions = new StringBuilder("position");
            for (ServoMove move : moves) positions.append(String.format(" %.3f", move.position(fraction)));
            return positions.toString();
        }
    }
}
