package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.hardware.AxonServo;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

/**
 * Finds every servo number in {@code TurretConstants} and {@code IntakeConstants}, one step at a time: the zero
 * positions, the directions, the travel limits, the feedback-wire voltages, the servo speeds and the move times.
 * Put the robot on a stand first.
 *
 * Gamepad 1:
 * <pre>
 *   dpad up/down       previous / next step (every servo goes limp when the step changes)
 *   left stick up/down move (pushing harder moves faster)
 *   dpad left/right    fine nudge (or trim / time, as the step says)
 *   A                  save the value (or run the automatic test)
 *   X / Y              answer the step's question (which way it moved), or start/stop a move-time test
 *   B                  go limp
 * </pre>
 * A servo stays limp until the first stick or dpad push, which powers it at the step's starting value without
 * moving it. Saved values go straight into the constants, so later steps (and other OpModes, until the robot app
 * restarts) use them. They are also written to {@code /sdcard/FIRST/ServoCalibration.txt}. Copy them into
 * the constants files; the last step lists them all.
 *
 * Steps that are not wired yet (a servo or feedback port missing from the robot configuration) are shown as
 * missing and skipped, so this works with only some servos connected.
 */
@TeleOp(name = "Servo Calibration", group = "Test")
public class ServoCalibration extends OpMode {
    public enum Mode {
        /** Dropdown stowed/deployed positions and an up/down test. Saves nothing: copy the numbers by hand. */
        DROPDOWN,
        /**
         * Each intake motor at a set RPM with its encoder, then both together, then the dropdown up/down test.
         * Saves nothing.
         */
        INTAKE,
        /** Turret servos A and B each on their own, both on separate sticks, then an angle test. Saves nothing. */
        TURRET,
        /** Every servo step. Saves into the constants as it goes (see the class comment). */
        ALL
    }

    /** Which steps to run. Change it here and reinstall. */
    public static Mode MODE = Mode.INTAKE;
    /** Intake test speed at the MOTOR shaft (the encoder is on the motor). Dpad left/right changes it live. */
    public static double INTAKE_TEST_RPM = 6000.0;
    public static double INTAKE_RPM_STEP = 100.0;
    /** goBILDA bare motor: 28 encoder ticks per motor revolution, about 6000 rpm free speed. */
    public static double MOTOR_TICKS_PER_REV = 28.0;
    public static double MOTOR_MAX_RPM = 6000.0;
    /**
     * Motor turns per roller turn, only for the roller RPM shown in telemetry. 1 = the current test setup, motors
     * direct on the shafts. The CAD belts are 64.0 / 24.0 (A) and 62.0 / 24.0 (B).
     */
    public static double INTAKE_A_RATIO = 24/64;
    public static double INTAKE_B_RATIO = 24/62;
    /** If the encoder reads under this share of the target after the grace time, the motor is stopped. */
    public static double NO_ENCODER_FRACTION = 0.05;
    public static double NO_ENCODER_GRACE_SEC = 0.5;
    /** Servo position change per second at full stick. The stick is squared, so small pushes are slow. */
    public static double JOG_RATE = 0.15;
    /** Servo position change per dpad press (0.002 = 0.7 deg of servo, 0.26 deg of turret). */
    public static double FINE_STEP = 0.002;
    public static double TRIM_STEP = 0.001;
    public static double TURRET_JOG_DEG_PER_SEC = 30.0;
    public static double TURRET_FINE_DEG = 0.5;
    public static double TILT_JOG_DEG_PER_SEC = 45.0;
    public static double TILT_FINE_DEG = 0.5;
    /** Feedback test: wait this long at each point, then average the voltage this long. */
    public static double SETTLE_SEC = 1.0;
    public static double SAMPLE_SEC = 0.3;
    /** The turret feedback test moves to this share of each travel limit. */
    public static double TURRET_TEST_FRACTION = 0.6;
    /** The tilt feedback test stays this share of the pitch range inside each end. */
    public static double TILT_TEST_MARGIN = 0.1;
    /** A middle point further than this (deg) from the straight line means the feedback isn't trustworthy. */
    public static double MAX_LINEARITY_ERROR_DEG = 2.0;
    public static double MOVE_TIME_STEP_SEC = 0.01;

    private static final double STICK_DEADBAND = 0.05;
    private static final String FILE_NAME = "ServoCalibration.txt";

    private final Set<String> missing = new LinkedHashSet<>();
    private final List<Srv> servos = new ArrayList<>();
    private final List<Step> steps = new ArrayList<>();
    private final List<Entry> entries = new ArrayList<>();
    private List<LynxModule> hubs;
    private Srv turretA;
    private Srv turretB;
    private Srv tilt;
    private Srv dropLeft;
    private Srv dropRight;

    private int index = 0;
    private String message = "";
    private long lastNanos = 0;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        if (MODE == Mode.ALL || MODE == Mode.TURRET) {
            turretA = servo(SERVO_A_NAME, FEEDBACK_NAME);
            turretB = servo(SERVO_B_NAME, "");
        }
        if (MODE == Mode.ALL) tilt = servo(TILT_SERVO_NAME, TILT_FEEDBACK_NAME);
        if (MODE != Mode.TURRET) {
            dropLeft = servo(DROPDOWN_LEFT_NAME, "");
            dropRight = servo(DROPDOWN_RIGHT_NAME, "");
        }
        allOff();
        addEntries();
        switch (MODE) {
            case DROPDOWN:
                addDropdownOnlySteps();
                steps.add(dropdownTestStep());
                break;
            case INTAKE:
                steps.add(new IntakeMotorStep("Intake motor A (roller bed)", MOTOR_A_NAME, MOTOR_A_REVERSED,
                        INTAKE_A_RATIO));
                steps.add(new IntakeMotorStep("Intake motor B (transfer)", MOTOR_B_NAME, MOTOR_B_REVERSED,
                        INTAKE_B_RATIO));
                steps.add(new IntakeMotorStep("Both intake motors together",
                        new String[]{"A", "B"}, new String[]{MOTOR_A_NAME, MOTOR_B_NAME},
                        new boolean[]{MOTOR_A_REVERSED, MOTOR_B_REVERSED},
                        new double[]{INTAKE_A_RATIO, INTAKE_B_RATIO}));
                steps.add(dropdownTestStep());
                break;
            case TURRET:
                addTurretOnlySteps();
                break;
            default:
                addTurretSteps();
                addTiltSteps();
                addDropdownPositionSteps();
                addDropdownMoveTimeStep();
                steps.add(new SummaryStep());
        }
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Put the robot on a stand, then press start.");
        telemetry.addData("Mode", MODE == Mode.ALL ? "ALL (saves as it goes)" : MODE + " (nothing is saved)");
        telemetry.addData("Steps", steps.size());
        telemetry.addData("Not in robot configuration", missing.isEmpty() ? "none" : missing.toString());
    }

    @Override
    public void start() {
        lastNanos = System.nanoTime();
        steps.get(index).enter();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;

        // The WasPressed methods clear themselves when read, so read each one exactly once per loop.
        boolean next = gamepad1.dpadDownWasPressed();
        boolean previous = gamepad1.dpadUpWasPressed();
        Input in = new Input();
        double stick = -gamepad1.left_stick_y;
        in.stick = Math.abs(stick) < STICK_DEADBAND ? 0 : stick * Math.abs(stick);
        double rightStick = -gamepad1.right_stick_y;
        in.rightStick = Math.abs(rightStick) < STICK_DEADBAND ? 0 : rightStick * Math.abs(rightStick);
        in.lb = gamepad1.leftBumperWasPressed();
        in.rb = gamepad1.rightBumperWasPressed();
        in.left = gamepad1.dpadLeftWasPressed();
        in.right = gamepad1.dpadRightWasPressed();
        in.a = gamepad1.aWasPressed();
        in.b = gamepad1.bWasPressed();
        in.x = gamepad1.xWasPressed();
        in.y = gamepad1.yWasPressed();

        Step step = steps.get(index);
        if (next || previous) {
            step.exit();
            index = Math.floorMod(index + (next ? 1 : -1), steps.size());
            message = "";
            step = steps.get(index);
            step.enter();
        } else if (step.missing().isEmpty()) {
            if (in.b) {
                step.exit();
                step.enter();
            } else {
                step.loop(in, dt);
            }
        }

        telemetry.addData("Step", "%d of %d: %s", index + 1, steps.size(), step.title);
        String stepMissing = step.missing();
        if (!stepMissing.isEmpty()) telemetry.addData("NOT IN ROBOT CONFIGURATION", stepMissing + " - skip this step");
        for (String line : step.help) telemetry.addLine(line);
        telemetry.addLine();
        step.addTelemetry();
        if (!message.isEmpty()) telemetry.addData("Result", message);
        telemetry.addLine("Dpad up/down = step, B = go limp");
    }

    @Override
    public void stop() {
        allOff();
        for (Step step : steps) step.exit();
        if (MODE == Mode.ALL) writeFile();
    }

    // ------------------------------------------------------------------------------------------------------------
    // Steps
    // ------------------------------------------------------------------------------------------------------------

    private void addTurretSteps() {
        steps.add(new JogStep("Turret servo A zero", new Srv[]{turretA},
                new String[]{
                        "Take servo B's gear off the turntable (or unplug B) so only A drives it.",
                        "Stick/dpad until the turret points straight forward. A = save SERVO_A_ZERO.",
                        "Then push the stick up: X if the turret turned LEFT, Y if it turned RIGHT."},
                JOG_RATE, FINE_STEP, 0, 1, () -> SERVO_A_ZERO,
                p -> turretA.set(p),
                p -> {
                    SERVO_A_ZERO = p;
                    return "SERVO_A_ZERO = " + f4(p);
                },
                () -> {
                    SERVO_A_DIRECTION = 1;
                    return "SERVO_A_DIRECTION = 1 (stick up = left)";
                },
                () -> {
                    SERVO_A_DIRECTION = -1;
                    return "SERVO_A_DIRECTION = -1 (stick up = right)";
                },
                p -> "servo A " + f4(p) + feedbackText(turretA)));

        steps.add(new JogStep("Turret servo B zero", new Srv[]{turretB},
                new String[]{
                        "Take servo A's gear off and put B's back on (or unplug A) so only B drives.",
                        "Stick/dpad until the turret points straight forward. A = save SERVO_B_ZERO.",
                        "Then push the stick up: X if the turret turned LEFT, Y if it turned RIGHT."},
                JOG_RATE, FINE_STEP, 0, 1, () -> SERVO_B_ZERO,
                p -> turretB.set(p),
                p -> {
                    SERVO_B_ZERO = p;
                    return "SERVO_B_ZERO = " + f4(p);
                },
                () -> {
                    SERVO_B_DIRECTION = 1;
                    return "SERVO_B_DIRECTION = 1 (stick up = left)" + directionCheck();
                },
                () -> {
                    SERVO_B_DIRECTION = -1;
                    return "SERVO_B_DIRECTION = -1 (stick up = right)" + directionCheck();
                },
                p -> "servo B " + f4(p)));

        steps.add(new PairStep("Turret both servos: trim B", new Srv[]{turretA, turretB},
                new String[]{
                        "Both gears on. Stick = turret angle. Dpad left/right = trim servo B's zero.",
                        "When the two servos disagree they fight: hub current goes up and they get warm.",
                        "Trim for the lowest current, checked at a few angles. A = save SERVO_B_ZERO."},
                TURRET_JOG_DEG_PER_SEC, -75, 75, TRIM_STEP, () -> 0.0, () -> SERVO_B_ZERO,
                (deg, bZero) -> {
                    turretA.set(turretPosition(SERVO_A_ZERO, SERVO_A_DIRECTION, deg));
                    turretB.set(turretPosition(bZero, SERVO_B_DIRECTION, deg));
                },
                (deg, bZero) -> {
                    SERVO_B_ZERO = bZero;
                    return "SERVO_B_ZERO = " + f4(bZero);
                },
                (deg, bZero) -> String.format(Locale.US, "turret %+.1f deg, B zero %.4f, hub current %.2f A",
                        deg, bZero, hubAmps())));

        steps.add(new JogStep("Turret left limit", new Srv[]{turretA},
                new String[]{
                        "Stick/dpad LEFT until the turret is as far as the wires and hard stops allow,",
                        "then back off a few degrees. A = save MAX_ANGLE_DEG."},
                TURRET_JOG_DEG_PER_SEC, TURRET_FINE_DEG, -75, 75, () -> 0.0,
                this::driveTurret,
                deg -> {
                    MAX_ANGLE_DEG = deg;
                    return "MAX_ANGLE_DEG = " + f1(deg);
                },
                null, null, this::turretStatus));

        steps.add(new JogStep("Turret right limit", new Srv[]{turretA},
                new String[]{
                        "Stick/dpad RIGHT until the turret is as far as the wires and hard stops allow,",
                        "then back off a few degrees. A = save MIN_ANGLE_DEG."},
                TURRET_JOG_DEG_PER_SEC, TURRET_FINE_DEG, -75, 75, () -> 0.0,
                this::driveTurret,
                deg -> {
                    MIN_ANGLE_DEG = deg;
                    return "MIN_ANGLE_DEG = " + f1(deg);
                },
                null, null, this::turretStatus));

        steps.add(new FeedbackStep("Turret feedback + speed", turretA,
                new String[]{
                        "Both gears on, zeros and limits done. A = run: the turret swings to the right,",
                        "then left, then center, reading servo A's feedback wire at each stop.",
                        "Saves FEEDBACK_VOLTS_AT_ZERO/ONE and SERVO_MAX_DEG_PER_SEC."},
                () -> new double[]{TURRET_TEST_FRACTION * MIN_ANGLE_DEG, TURRET_TEST_FRACTION * MAX_ANGLE_DEG, 0},
                this::driveTurret,
                deg -> turretPosition(SERVO_A_ZERO, SERVO_A_DIRECTION, deg),
                (v0, v1, rate) -> {
                    FEEDBACK_VOLTS_AT_ZERO = v0;
                    FEEDBACK_VOLTS_AT_ONE = v1;
                    if (!Double.isNaN(rate)) SERVO_MAX_DEG_PER_SEC = rate;
                }));
    }

    private void addTiltSteps() {
        steps.add(new JogStep("Camera tilt level", new Srv[]{tilt},
                new String[]{
                        "Put a level (or a phone level app) on the camera.",
                        "Stick/dpad until the lens points level. A = save TILT_SERVO_LEVEL.",
                        "Then push the stick up: X if the camera tilted UP, Y if it tilted DOWN."},
                JOG_RATE, FINE_STEP, 0, 1, () -> TILT_SERVO_LEVEL,
                p -> tilt.set(p),
                p -> {
                    TILT_SERVO_LEVEL = p;
                    return "TILT_SERVO_LEVEL = " + f4(p);
                },
                () -> {
                    TILT_DIRECTION = 1;
                    return "TILT_DIRECTION = 1 (stick up = tilt up)";
                },
                () -> {
                    TILT_DIRECTION = -1;
                    return "TILT_DIRECTION = -1 (stick up = tilt down)";
                },
                p -> "tilt servo " + f4(p) + feedbackText(tilt)));

        steps.add(new JogStep("Camera tilt up limit", new Srv[]{tilt},
                new String[]{
                        "Stick/dpad UP until the camera arm nearly touches something, then back off.",
                        "A = save MAX_PITCH_DEG."},
                TILT_JOG_DEG_PER_SEC, TILT_FINE_DEG, -30, 120, () -> DEFAULT_PITCH_DEG,
                pitch -> tilt.set(tiltPosition(pitch)),
                pitch -> {
                    MAX_PITCH_DEG = pitch;
                    return "MAX_PITCH_DEG = " + f1(pitch);
                },
                null, null, this::tiltStatus));

        steps.add(new JogStep("Camera tilt down limit", new Srv[]{tilt},
                new String[]{
                        "Stick/dpad DOWN until the camera arm nearly touches something, then back off.",
                        "A = save MIN_PITCH_DEG."},
                TILT_JOG_DEG_PER_SEC, TILT_FINE_DEG, -30, 120, () -> DEFAULT_PITCH_DEG,
                pitch -> tilt.set(tiltPosition(pitch)),
                pitch -> {
                    MIN_PITCH_DEG = pitch;
                    return "MIN_PITCH_DEG = " + f1(pitch);
                },
                null, null, this::tiltStatus));

        steps.add(new FeedbackStep("Camera tilt feedback + speed", tilt,
                new String[]{
                        "Level and limits done. A = run: the camera tilts down, up, then to the middle,",
                        "reading the tilt servo's feedback wire at each stop.",
                        "Saves TILT_FEEDBACK_VOLTS_AT_ZERO/ONE and TILT_MAX_DEG_PER_SEC."},
                () -> {
                    double margin = TILT_TEST_MARGIN * (MAX_PITCH_DEG - MIN_PITCH_DEG);
                    return new double[]{MIN_PITCH_DEG + margin, MAX_PITCH_DEG - margin,
                            (MIN_PITCH_DEG + MAX_PITCH_DEG) / 2};
                },
                pitch -> tilt.set(tiltPosition(pitch)),
                ServoCalibration::tiltPosition,
                (v0, v1, rate) -> {
                    TILT_FEEDBACK_VOLTS_AT_ZERO = v0;
                    TILT_FEEDBACK_VOLTS_AT_ONE = v1;
                    if (!Double.isNaN(rate)) TILT_MAX_DEG_PER_SEC = rate;
                }));
    }

    /**
     * Dropdown up and down positions with nothing saved. Each step remembers where you left it while the OpMode
     * runs, and the down step starts from the two up positions you found.
     */
    private void addDropdownOnlySteps() {
        final double[] leftUp = {DROPDOWN_LEFT_STOWED};
        final double[] rightUp = {DROPDOWN_RIGHT_STOWED};
        final double[] leftDown = {Double.NaN};
        final double[] trim = {0};

        steps.add(new JogStep("Dropdown left up", new Srv[]{dropLeft},
                new String[]{
                        "The right servo is limp. Stick/dpad the LEFT servo until the dropdown is fully",
                        "up (inside 18 in.) without straining. Copy the number into IntakeConstants."},
                JOG_RATE, FINE_STEP, 0, 1, () -> leftUp[0],
                p -> {
                    leftUp[0] = p;
                    dropLeft.set(p);
                },
                null, null, null,
                p -> "DROPDOWN_LEFT_STOWED = " + f4(p)));

        steps.add(new JogStep("Dropdown right up", new Srv[]{dropRight},
                new String[]{
                        "The left servo is limp. Stick/dpad the RIGHT servo until the dropdown is fully",
                        "up (inside 18 in.) without straining. Copy the number into IntakeConstants."},
                JOG_RATE, FINE_STEP, 0, 1, () -> rightUp[0],
                p -> {
                    rightUp[0] = p;
                    dropRight.set(p);
                },
                null, null, null,
                p -> "DROPDOWN_RIGHT_STOWED = " + f4(p)));

        steps.add(new PairStep("Dropdown down", new Srv[]{dropLeft, dropRight},
                new String[]{
                        "Starts at the two up positions. Stick = both servos together (mirrored) until the",
                        "dropdown is down. Dpad left/right = trim the right servo so there is no twist.",
                        "Copy both numbers into IntakeConstants."},
                JOG_RATE, 0, 1, TRIM_STEP,
                () -> Double.isNaN(leftDown[0]) ? leftUp[0] : leftDown[0],
                () -> trim[0],
                (left, t) -> {
                    leftDown[0] = left;
                    trim[0] = t;
                    dropLeft.set(left);
                    dropRight.set(rightUp[0] + dropdownMirror() * (left - leftUp[0]) + t);
                },
                null,
                (left, t) -> "DROPDOWN_LEFT_DEPLOYED = " + f4(left) + ", DROPDOWN_RIGHT_DEPLOYED = "
                        + f4(Range.clip(rightUp[0] + dropdownMirror() * (left - leftUp[0]) + t, 0, 1))));
    }

    /**
     * Turret servos with nothing saved: A alone, B alone, both on separate sticks, then an angle test that uses
     * TurretConstants. Steps 1-3 share the two positions, so each picks up where the last one left them.
     */
    private void addTurretOnlySteps() {
        final double[] a = {SERVO_A_ZERO};
        final double[] b = {SERVO_B_ZERO};

        steps.add(new JogStep("Turret servo A alone", new Srv[]{turretA},
                new String[]{
                        "Servo B is limp. Take B's gear off the turntable (or unplug B) so only A drives.",
                        "Stick/dpad until the turret points straight forward: that number is SERVO_A_ZERO.",
                        "Stick up raises the number. Turret turns left = SERVO_A_DIRECTION 1, right = -1."},
                JOG_RATE, FINE_STEP, 0, 1, () -> a[0],
                p -> {
                    a[0] = p;
                    turretA.set(p);
                },
                null, null, null,
                p -> "servo A " + f4(p) + feedbackText(turretA)));

        steps.add(new JogStep("Turret servo B alone", new Srv[]{turretB},
                new String[]{
                        "Servo A is limp. Take A's gear off (or unplug A) and put B's on so only B drives.",
                        "Stick/dpad until the turret points straight forward: that number is SERVO_B_ZERO.",
                        "Stick up raises the number. Turret turns left = SERVO_B_DIRECTION 1, right = -1."},
                JOG_RATE, FINE_STEP, 0, 1, () -> b[0],
                p -> {
                    b[0] = p;
                    turretB.set(p);
                },
                null, null, null,
                p -> "servo B " + f4(p)));

        steps.add(new TwoServoStep(a, b));

        steps.add(new JogStep("Turret angle test", new Srv[]{turretA},
                new String[]{
                        "Uses TurretConstants (zeros, directions, limits): rebuild and reinstall after changing them.",
                        "Both gears on. Stick/dpad = turret angle, kept inside MIN/MAX_ANGLE_DEG.",
                        "0 deg should point straight forward, with quiet servos and low hub current."},
                TURRET_JOG_DEG_PER_SEC, TURRET_FINE_DEG, MIN_ANGLE_DEG, MAX_ANGLE_DEG, () -> 0.0,
                this::driveTurret,
                null, null, null,
                deg -> turretStatus(deg) + String.format(Locale.US, ", hub current %.2f A", hubAmps())));
    }

    private void addDropdownPositionSteps() {
        steps.add(new JogStep("Dropdown left stowed", new Srv[]{dropLeft},
                new String[]{
                        "The right servo is limp. Stick/dpad the LEFT servo until the dropdown is fully",
                        "stowed (inside 18 in.) without straining. A = save DROPDOWN_LEFT_STOWED."},
                JOG_RATE, FINE_STEP, 0, 1, () -> DROPDOWN_LEFT_STOWED,
                p -> dropLeft.set(p),
                p -> {
                    DROPDOWN_LEFT_STOWED = p;
                    return "DROPDOWN_LEFT_STOWED = " + f4(p);
                },
                null, null, p -> "left servo " + f4(p)));

        steps.add(new JogStep("Dropdown right stowed", new Srv[]{dropRight},
                new String[]{
                        "The left servo is limp. Stick/dpad the RIGHT servo until the dropdown is fully",
                        "stowed (inside 18 in.) without straining. A = save DROPDOWN_RIGHT_STOWED."},
                JOG_RATE, FINE_STEP, 0, 1, () -> DROPDOWN_RIGHT_STOWED,
                p -> dropRight.set(p),
                p -> {
                    DROPDOWN_RIGHT_STOWED = p;
                    return "DROPDOWN_RIGHT_STOWED = " + f4(p);
                },
                null, null, p -> "right servo " + f4(p)));

        steps.add(new PairStep("Dropdown deployed", new Srv[]{dropLeft, dropRight},
                new String[]{
                        "Starts stowed. Stick = both servos together (mirrored) until the dropdown is down.",
                        "Dpad left/right = trim the right servo so both sides sit evenly with no twist.",
                        "A = save DROPDOWN_LEFT_DEPLOYED and DROPDOWN_RIGHT_DEPLOYED."},
                JOG_RATE, 0, 1, TRIM_STEP, () -> DROPDOWN_LEFT_STOWED, () -> 0.0,
                (left, trim) -> {
                    dropLeft.set(left);
                    dropRight.set(dropdownRight(left, trim));
                },
                (left, trim) -> {
                    double right = Range.clip(dropdownRight(left, trim), 0, 1);
                    DROPDOWN_LEFT_DEPLOYED = left;
                    DROPDOWN_RIGHT_DEPLOYED = right;
                    return "DROPDOWN_LEFT_DEPLOYED = " + f4(left) + ", DROPDOWN_RIGHT_DEPLOYED = " + f4(right);
                },
                (left, trim) -> "left " + f4(left) + ", right " + f4(Range.clip(dropdownRight(left, trim), 0, 1))));
    }

    private void addDropdownMoveTimeStep() {
        steps.add(new MoveTimeStep("Dropdown move time", new Srv[]{dropLeft, dropRight}, "DROPDOWN_MOVE_SEC",
                () -> DROPDOWN_MOVE_SEC, sec -> DROPDOWN_MOVE_SEC = sec,
                down -> {
                    dropLeft.set(down ? DROPDOWN_LEFT_DEPLOYED : DROPDOWN_LEFT_STOWED);
                    dropRight.set(down ? DROPDOWN_RIGHT_DEPLOYED : DROPDOWN_RIGHT_STOWED);
                }));
    }

    // ------------------------------------------------------------------------------------------------------------
    // Servo math (same formulas as Turret, CameraTilt and Dropdown)
    // ------------------------------------------------------------------------------------------------------------

    private static double turretPosition(double zero, double direction, double turretDeg) {
        return zero + turretDeg * TURRET_GEAR_RATIO / (direction * AxonServo.RANGE_DEG);
    }

    private static double tiltPosition(double pitchDeg) {
        return TILT_SERVO_LEVEL + pitchDeg / (TILT_DIRECTION * AxonServo.RANGE_DEG);
    }

    /** The right dropdown servo mirrors the left one around their stowed positions, plus a trim. */
    private static double dropdownRight(double left, double trim) {
        return DROPDOWN_RIGHT_STOWED + dropdownMirror() * (left - DROPDOWN_LEFT_STOWED) + trim;
    }

    /** -1 when the right servo's numbers run opposite to the left's (they face each other), from the constants. */
    private static double dropdownMirror() {
        double leftTravel = DROPDOWN_LEFT_DEPLOYED - DROPDOWN_LEFT_STOWED;
        double rightTravel = DROPDOWN_RIGHT_DEPLOYED - DROPDOWN_RIGHT_STOWED;
        return leftTravel == 0 || rightTravel == 0 ? -1 : Math.signum(leftTravel * rightTravel);
    }

    private void driveTurret(double deg) {
        turretA.set(turretPosition(SERVO_A_ZERO, SERVO_A_DIRECTION, deg));
        turretB.set(turretPosition(SERVO_B_ZERO, SERVO_B_DIRECTION, deg));
    }

    private String turretStatus(double deg) {
        double a = turretPosition(SERVO_A_ZERO, SERVO_A_DIRECTION, deg);
        double b = turretPosition(SERVO_B_ZERO, SERVO_B_DIRECTION, deg);
        boolean atEnd = a <= 0 || a >= 1 || (turretB.found() && (b <= 0 || b >= 1));
        return String.format(Locale.US, "turret %+.1f deg, servo A %.3f, B %.3f%s%s", deg, a, b,
                feedbackText(turretA), atEnd ? "  AT END OF SERVO RANGE" : "");
    }

    private String tiltStatus(double pitch) {
        double p = tiltPosition(pitch);
        return String.format(Locale.US, "pitch %.1f deg, servo %.3f%s%s", pitch, p, feedbackText(tilt),
                p <= 0 || p >= 1 ? "  AT END OF SERVO RANGE" : "");
    }

    private static String directionCheck() {
        return SERVO_A_DIRECTION == SERVO_B_DIRECTION ? "" : "  (differs from servo A - check before using both)";
    }

    private double hubAmps() {
        double amps = 0;
        for (LynxModule hub : hubs) amps += hub.getCurrent(CurrentUnit.AMPS);
        return amps;
    }

    // ------------------------------------------------------------------------------------------------------------
    // Hardware and output
    // ------------------------------------------------------------------------------------------------------------

    private Srv servo(String name, String feedbackName) {
        ServoImplEx servo = hardwareMap.tryGet(ServoImplEx.class, name);
        if (servo == null) {
            missing.add(name);
        } else {
            servo.setPwmRange(new PwmControl.PwmRange(AxonServo.PWM_MIN_US, AxonServo.PWM_MAX_US));
        }
        AnalogInput feedback = null;
        if (!feedbackName.isEmpty()) {
            feedback = hardwareMap.tryGet(AnalogInput.class, feedbackName);
            if (feedback == null) missing.add(feedbackName);
        }
        Srv srv = new Srv(name, feedbackName, servo, feedback);
        servos.add(srv);
        return srv;
    }

    private void allOff() {
        for (Srv servo : servos) servo.off();
    }

    private static String feedbackText(Srv servo) {
        return servo.feedback == null ? "" : String.format(Locale.US, ", feedback %.3f V", servo.volts());
    }

    private void addEntries() {
        if (MODE != Mode.ALL) return;
        String i = "IntakeConstants";
        String t = "TurretConstants";
        entries.add(new Entry(t, "SERVO_A_ZERO", "%.4f", () -> SERVO_A_ZERO));
        entries.add(new Entry(t, "SERVO_B_ZERO", "%.4f", () -> SERVO_B_ZERO));
        entries.add(new Entry(t, "SERVO_A_DIRECTION", "%.1f", () -> SERVO_A_DIRECTION));
        entries.add(new Entry(t, "SERVO_B_DIRECTION", "%.1f", () -> SERVO_B_DIRECTION));
        entries.add(new Entry(t, "FEEDBACK_VOLTS_AT_ZERO", "%.3f", () -> FEEDBACK_VOLTS_AT_ZERO));
        entries.add(new Entry(t, "FEEDBACK_VOLTS_AT_ONE", "%.3f", () -> FEEDBACK_VOLTS_AT_ONE));
        entries.add(new Entry(t, "MIN_ANGLE_DEG", "%.1f", () -> MIN_ANGLE_DEG));
        entries.add(new Entry(t, "MAX_ANGLE_DEG", "%.1f", () -> MAX_ANGLE_DEG));
        entries.add(new Entry(t, "SERVO_MAX_DEG_PER_SEC", "%.0f", () -> SERVO_MAX_DEG_PER_SEC));
        entries.add(new Entry(t, "TILT_SERVO_LEVEL", "%.4f", () -> TILT_SERVO_LEVEL));
        entries.add(new Entry(t, "TILT_DIRECTION", "%.1f", () -> TILT_DIRECTION));
        entries.add(new Entry(t, "TILT_FEEDBACK_VOLTS_AT_ZERO", "%.3f", () -> TILT_FEEDBACK_VOLTS_AT_ZERO));
        entries.add(new Entry(t, "TILT_FEEDBACK_VOLTS_AT_ONE", "%.3f", () -> TILT_FEEDBACK_VOLTS_AT_ONE));
        entries.add(new Entry(t, "MIN_PITCH_DEG", "%.1f", () -> MIN_PITCH_DEG));
        entries.add(new Entry(t, "MAX_PITCH_DEG", "%.1f", () -> MAX_PITCH_DEG));
        entries.add(new Entry(t, "TILT_MAX_DEG_PER_SEC", "%.0f", () -> TILT_MAX_DEG_PER_SEC));
        entries.add(new Entry(i, "DROPDOWN_LEFT_STOWED", "%.4f", () -> DROPDOWN_LEFT_STOWED));
        entries.add(new Entry(i, "DROPDOWN_LEFT_DEPLOYED", "%.4f", () -> DROPDOWN_LEFT_DEPLOYED));
        entries.add(new Entry(i, "DROPDOWN_RIGHT_STOWED", "%.4f", () -> DROPDOWN_RIGHT_STOWED));
        entries.add(new Entry(i, "DROPDOWN_RIGHT_DEPLOYED", "%.4f", () -> DROPDOWN_RIGHT_DEPLOYED));
        entries.add(new Entry(i, "DROPDOWN_MOVE_SEC", "%.2f", () -> DROPDOWN_MOVE_SEC));
    }

    /** Writes every value as a paste-ready line to /sdcard/FIRST/ServoCalibration.txt. */
    private void writeFile() {
        StringBuilder text = new StringBuilder("// Servo Calibration results. \"was\" = changed in this run.\n");
        String currentClass = "";
        for (Entry entry : entries) {
            if (!entry.className.equals(currentClass)) {
                currentClass = entry.className;
                text.append("\n// ").append(currentClass).append(".java\n");
            }
            text.append("public static double ").append(entry.name).append(" = ").append(entry.value()).append(';');
            if (entry.changed()) text.append("   // was ").append(entry.format(entry.original));
            text.append('\n');
        }
        File file = new File(AppUtil.FIRST_FOLDER, FILE_NAME);
        try (FileWriter writer = new FileWriter(file)) {
            writer.write(text.toString());
        } catch (IOException e) {
            message = "Could not write " + file + ": " + e.getMessage();
        }
    }

    private void saved(String result) {
        message = result;
        writeFile();
    }

    private static String f1(double value) {
        return String.format(Locale.US, "%.1f", value);
    }

    private static String f4(double value) {
        return String.format(Locale.US, "%.4f", value);
    }

    // ------------------------------------------------------------------------------------------------------------
    // Helper types
    // ------------------------------------------------------------------------------------------------------------

    private static class Input {
        /** Left stick up, squared, with a deadband. */
        double stick;
        /** Right stick up, squared, with a deadband. */
        double rightStick;
        boolean left, right, a, b, x, y, lb, rb;
    }

    /** One servo (null if it is not in the robot configuration) and its optional feedback wire. */
    private static class Srv {
        final String name;
        final String feedbackName;
        final ServoImplEx servo;
        final AnalogInput feedback;
        private boolean enabled = true;

        Srv(String name, String feedbackName, ServoImplEx servo, AnalogInput feedback) {
            this.name = name;
            this.feedbackName = feedbackName;
            this.servo = servo;
            this.feedback = feedback;
        }

        boolean found() {
            return servo != null;
        }

        void set(double position) {
            if (servo == null) return;
            if (!enabled) {
                servo.setPwmEnable();
                enabled = true;
            }
            servo.setPosition(Range.clip(position, 0, 1));
        }

        void off() {
            if (servo == null) return;
            servo.setPwmDisable();
            enabled = false;
        }

        double volts() {
            return feedback == null ? Double.NaN : feedback.getVoltage();
        }
    }

    private static class Entry {
        final String className;
        final String name;
        final String pattern;
        final DoubleSupplier getter;
        final double original;

        Entry(String className, String name, String pattern, DoubleSupplier getter) {
            this.className = className;
            this.name = name;
            this.pattern = pattern;
            this.getter = getter;
            this.original = getter.getAsDouble();
        }

        String format(double value) {
            return String.format(Locale.US, pattern, value);
        }

        String value() {
            return format(getter.getAsDouble());
        }

        boolean changed() {
            return !format(original).equals(value());
        }
    }

    private interface FeedbackSave {
        void save(double voltsAtZero, double voltsAtOne, double degPerSec);
    }

    private abstract class Step {
        final String title;
        final Srv[] uses;
        final String[] help;

        Step(String title, Srv[] uses, String[] help) {
            this.title = title;
            this.uses = uses;
            this.help = help;
        }

        /** Names of the devices this step needs that are not in the robot configuration, or "". */
        String missing() {
            StringBuilder names = new StringBuilder();
            for (Srv servo : uses) {
                if (!servo.found()) names.append(names.length() == 0 ? "" : ", ").append(servo.name);
            }
            return names.toString();
        }

        void enter() {}

        abstract void loop(Input in, double dt);

        abstract void addTelemetry();

        void exit() {
            allOff();
        }
    }

    /** Move one value with the stick and dpad; A saves it, X/Y answer an optional question. */
    private class JogStep extends Step {
        final double rate, fine, min, max;
        final DoubleSupplier start;
        final DoubleConsumer drive;
        final DoubleFunction<String> save;
        final Supplier<String> onX, onY;
        final DoubleFunction<String> status;
        double value;
        boolean powered;

        JogStep(String title, Srv[] uses, String[] help, double rate, double fine, double min, double max,
                DoubleSupplier start, DoubleConsumer drive, DoubleFunction<String> save,
                Supplier<String> onX, Supplier<String> onY, DoubleFunction<String> status) {
            super(title, uses, help);
            this.rate = rate;
            this.fine = fine;
            this.min = min;
            this.max = max;
            this.start = start;
            this.drive = drive;
            this.save = save;
            this.onX = onX;
            this.onY = onY;
            this.status = status;
        }

        @Override
        void enter() {
            value = start.getAsDouble();
            powered = false;
        }

        @Override
        void loop(Input in, double dt) {
            boolean moved = in.stick != 0 || in.left || in.right;
            if (moved && !powered) {
                powered = true;
            } else if (powered) {
                value += in.stick * rate * dt + (in.right ? fine : 0) - (in.left ? fine : 0);
                value = Range.clip(value, min, max);
            }
            if (powered) drive.accept(value);
            if (in.a && powered && save != null) saved(save.apply(value));
            if (in.x && onX != null) saved(onX.get());
            if (in.y && onY != null) saved(onY.get());
        }

        @Override
        void addTelemetry() {
            telemetry.addData("Now", (powered ? "" : "LIMP (push the stick or dpad to power it here) ")
                    + status.apply(value));
        }
    }

    /** Stick moves a main value, dpad trims a second one; A saves. */
    private class PairStep extends Step {
        final double rate, min, max, trimStep;
        final DoubleSupplier mainStart, trimStart;
        final BiConsumer<Double, Double> drive;
        final BiFunction<Double, Double, String> save, status;
        double main, trim;
        boolean powered;

        PairStep(String title, Srv[] uses, String[] help, double rate, double min, double max, double trimStep,
                 DoubleSupplier mainStart, DoubleSupplier trimStart, BiConsumer<Double, Double> drive,
                 BiFunction<Double, Double, String> save, BiFunction<Double, Double, String> status) {
            super(title, uses, help);
            this.rate = rate;
            this.min = min;
            this.max = max;
            this.trimStep = trimStep;
            this.mainStart = mainStart;
            this.trimStart = trimStart;
            this.drive = drive;
            this.save = save;
            this.status = status;
        }

        @Override
        void enter() {
            main = mainStart.getAsDouble();
            trim = trimStart.getAsDouble();
            powered = false;
        }

        @Override
        void loop(Input in, double dt) {
            boolean moved = in.stick != 0 || in.left || in.right;
            if (moved && !powered) {
                powered = true;
            } else if (powered) {
                main = Range.clip(main + in.stick * rate * dt, min, max);
                trim += (in.right ? trimStep : 0) - (in.left ? trimStep : 0);
            }
            if (powered) drive.accept(main, trim);
            if (in.a && powered && save != null) saved(save.apply(main, trim));
        }

        @Override
        void addTelemetry() {
            telemetry.addData("Now", powered ? status.apply(main, trim)
                    : "limp - push the stick or dpad to power it at the start position");
        }
    }

    /**
     * Automatic: drives to points 0, 1 and 2 (in degrees), averages the feedback voltage at each, fits a line
     * through points 0 and 1 (in servo position) and extrapolates the voltage at positions 0 and 1. Point 2 checks
     * that the line holds. The move from point 0 to point 1 is timed (10% to 90% of the voltage change) for speed.
     */
    private class FeedbackStep extends Step {
        final Srv sensor;
        final Supplier<double[]> plan;
        final DoubleConsumer drive;
        final DoubleUnaryOperator positionOf;
        final FeedbackSave save;
        final double[] volts = new double[3];
        final List<double[]> trace = new ArrayList<>();
        double[] points;
        int phase = -1;
        double time, sum;
        int samples;
        String result = "not run yet";

        FeedbackStep(String title, Srv sensor, String[] help, Supplier<double[]> plan, DoubleConsumer drive,
                     DoubleUnaryOperator positionOf, FeedbackSave save) {
            super(title, new Srv[]{sensor}, help);
            this.sensor = sensor;
            this.plan = plan;
            this.drive = drive;
            this.positionOf = positionOf;
            this.save = save;
        }

        @Override
        String missing() {
            String servos = super.missing();
            if (sensor.feedback != null) return servos;
            String wire = sensor.feedbackName.isEmpty() ? sensor.name + " feedback (no port name set)"
                    : sensor.feedbackName;
            return servos.isEmpty() ? wire : servos + ", " + wire;
        }

        @Override
        void enter() {
            phase = -1;
        }

        @Override
        void loop(Input in, double dt) {
            if (in.a && phase < 0) {
                points = plan.get();
                phase = 0;
                time = 0;
                sum = 0;
                samples = 0;
                trace.clear();
                result = "running";
            }
            if (phase < 0) return;

            time += dt;
            drive.accept(points[phase]);
            double v = sensor.volts();
            if (phase == 1) trace.add(new double[]{time, v});
            if (time >= SETTLE_SEC) {
                sum += v;
                samples++;
            }
            if (time >= SETTLE_SEC + SAMPLE_SEC && samples > 0) {
                volts[phase] = sum / samples;
                phase++;
                time = 0;
                sum = 0;
                samples = 0;
                if (phase == 3) {
                    phase = -1;
                    finish();
                }
            }
        }

        private void finish() {
            double dv = volts[1] - volts[0];
            if (Math.abs(dv) < 0.05) {
                result = String.format(Locale.US, "FAILED: voltage barely changed (%.3f to %.3f V). Check the"
                        + " feedback wire and analog port.", volts[0], volts[1]);
                message = result;
                return;
            }
            double p0 = positionOf.applyAsDouble(points[0]);
            double p1 = positionOf.applyAsDouble(points[1]);
            double slope = dv / (p1 - p0);
            double voltsAtZero = volts[0] - slope * p0;
            double voltsAtOne = voltsAtZero + slope;
            double middle = points[0] + (volts[2] - volts[0]) / dv * (points[1] - points[0]);
            double middleError = middle - points[2];
            double rate = slewRate(dv);

            save.save(voltsAtZero, voltsAtOne, rate);
            result = String.format(Locale.US, "V at 0 / 1 = %.3f / %.3f, middle off by %.1f deg, speed %s",
                    voltsAtZero, voltsAtOne, middleError,
                    Double.isNaN(rate) ? "too fast to time (kept old value)"
                            : String.format(Locale.US, "%.0f deg/s", rate));
            if (Math.abs(middleError) > MAX_LINEARITY_ERROR_DEG) {
                result += ". WARNING: not a straight line - gear slipping, servo straining or the feedback wrapped."
                        + " Fix it and run again.";
            }
            saved("saved. " + result);
        }

        private double slewRate(double dv) {
            double t10 = Double.NaN;
            double t90 = Double.NaN;
            for (double[] sample : trace) {
                double progress = (sample[1] - volts[0]) / dv;
                if (Double.isNaN(t10) && progress >= 0.1) t10 = sample[0];
                if (progress >= 0.9) {
                    t90 = sample[0];
                    break;
                }
            }
            if (Double.isNaN(t10) || Double.isNaN(t90) || t90 <= t10) return Double.NaN;
            return 0.8 * Math.abs(points[1] - points[0]) / (t90 - t10);
        }

        @Override
        void addTelemetry() {
            if (phase >= 0) {
                telemetry.addData("Now", "point %d of 3: %.1f deg, feedback %.3f V", phase + 1, points[phase],
                        sensor.volts());
            } else {
                telemetry.addData("Now", "A = run" + feedbackText(sensor));
            }
            telemetry.addData("Last run", result);
        }
    }

    /**
     * Swings a mechanism between its two positions, waiting the move time at each end. If the time is too short
     * the mechanism turns around before reaching the end. Shorten it until that happens, then lengthen it until it
     * just reaches both ends every time.
     */
    private class MoveTimeStep extends Step {
        final String constant;
        final DoubleSupplier getter;
        final DoubleConsumer setter;
        final Consumer<Boolean> drive;
        double sec, sinceSwitch;
        boolean running, atSecond;

        /** @param setter null = nothing is saved; the time is shown to copy by hand. */
        MoveTimeStep(String title, Srv[] uses, String constant, DoubleSupplier getter, DoubleConsumer setter,
                     Consumer<Boolean> drive) {
            super(title, uses, new String[]{
                    "X = start/stop swinging between the two positions, pausing the move time at each.",
                    "Dpad left/right = shorter/longer. Shorten until it turns around before reaching an end,",
                    "then lengthen until it just reaches both ends every time, and add about 0.02.",
                    setter == null ? "Copy " + constant + " into IntakeConstants by hand." : "A = save " + constant + "."});
            this.constant = constant;
            this.getter = getter;
            this.setter = setter;
            this.drive = drive;
        }

        @Override
        void enter() {
            sec = getter.getAsDouble();
            running = false;
        }

        @Override
        void loop(Input in, double dt) {
            if (in.x) {
                running = !running;
                atSecond = false;
                sinceSwitch = 0;
                if (running) drive.accept(false);
            }
            if (in.right) sec += MOVE_TIME_STEP_SEC;
            if (in.left) sec = Math.max(MOVE_TIME_STEP_SEC, sec - MOVE_TIME_STEP_SEC);
            if (running) {
                sinceSwitch += dt;
                if (sinceSwitch >= sec) {
                    atSecond = !atSecond;
                    sinceSwitch = 0;
                    drive.accept(atSecond);
                }
            }
            if (in.a && setter != null) {
                setter.accept(sec);
                saved(constant + " = " + String.format(Locale.US, "%.2f", sec));
            }
        }

        @Override
        void addTelemetry() {
            telemetry.addData("Now", "%s, %s = %.2f", running ? "swinging" : "stopped (X = start)", constant, sec);
        }
    }

    /**
     * Runs one or more intake motors at a set motor RPM using the hub's built-in velocity loop and each motor's
     * encoder. Without encoder counts that loop would push to full power, so a motor stops itself if its encoder
     * reads (almost) nothing, or counts the wrong way, after {@link #NO_ENCODER_GRACE_SEC}. With several motors,
     * each one is checked on its own and the others keep running.
     */
    private class IntakeMotorStep extends Step {
        private class Channel {
            final String label;
            final String name;
            final DcMotorEx motor;
            final double ratio;
            boolean running;
            String fault = "";

            Channel(String label, String name, boolean reversed, double ratio) {
                this.label = label;
                this.name = name;
                this.ratio = ratio;
                motor = hardwareMap.tryGet(DcMotorEx.class, name);
                if (motor == null) {
                    missing.add(name);
                } else {
                    motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motor.setPower(0);
                }
            }

            double measuredRpm() {
                return motor.getVelocity() * 60.0 / MOTOR_TICKS_PER_REV;
            }

            void stop() {
                running = false;
                if (motor != null) motor.setPower(0);
            }
        }

        final Channel[] channels;
        /** +1 toward the shooter, -1 reversed (eject), 0 stopped. */
        int direction = 0;
        double rpm = INTAKE_TEST_RPM;
        double runningSec;

        /** One motor. */
        IntakeMotorStep(String title, String motorName, boolean reversed, double ratio) {
            this(title, new String[]{""}, new String[]{motorName}, new boolean[]{reversed}, new double[]{ratio});
        }

        /** Several motors at the same target RPM; labels name each one in telemetry. */
        IntakeMotorStep(String title, String[] labels, String[] motorNames, boolean[] reversed, double[] ratios) {
            super(title, new Srv[0], new String[]{
                    "Robot on a stand, hands clear. "
                            + (motorNames.length == 1 ? "Plug the encoder cable into this motor's port."
                            : "Every motor in this step needs its encoder cable plugged in."),
                    "A = run toward the shooter, Y = run reversed (eject), B = stop.",
                    "Dpad left/right = target -/+ " + (int) INTAKE_RPM_STEP + " rpm (motor shaft)."});
            channels = new Channel[motorNames.length];
            for (int i = 0; i < motorNames.length; i++) {
                channels[i] = new Channel(labels[i], motorNames[i], reversed[i], ratios[i]);
            }
        }

        @Override
        String missing() {
            StringBuilder names = new StringBuilder();
            for (Channel c : channels) {
                if (c.motor == null) names.append(names.length() == 0 ? "" : ", ").append(c.name);
            }
            return names.toString();
        }

        @Override
        void enter() {
            stopAll();
            for (Channel c : channels) c.fault = "";
        }

        @Override
        void loop(Input in, double dt) {
            if (in.right) rpm = Math.min(MOTOR_MAX_RPM, rpm + INTAKE_RPM_STEP);
            if (in.left) rpm = Math.max(INTAKE_RPM_STEP, rpm - INTAKE_RPM_STEP);
            if (in.a) start(1);
            if (in.y) start(-1);
            if (direction == 0) return;

            runningSec += dt;
            boolean anyRunning = false;
            for (Channel c : channels) {
                if (!c.running) continue;
                c.motor.setVelocity(direction * rpm * MOTOR_TICKS_PER_REV / 60.0);
                double measured = c.measuredRpm();
                if (runningSec > NO_ENCODER_GRACE_SEC && measured * direction < NO_ENCODER_FRACTION * rpm) {
                    c.stop();
                    c.fault = String.format(Locale.US, "STOPPED: encoder read %.0f rpm with a target of %d. Check"
                            + " that the encoder cable is in this motor's port and counts the same way the motor"
                            + " turns.", measured, (int) (direction * rpm));
                } else {
                    anyRunning = true;
                }
            }
            if (!anyRunning) direction = 0;
        }

        private void start(int newDirection) {
            direction = newDirection;
            runningSec = 0;
            for (Channel c : channels) {
                c.running = true;
                c.fault = "";
            }
        }

        private void stopAll() {
            direction = 0;
            runningSec = 0;
            for (Channel c : channels) c.stop();
        }

        @Override
        void exit() {
            stopAll();
            super.exit();
        }

        @Override
        void addTelemetry() {
            telemetry.addData("Now", direction == 0 ? "stopped (A = run, Y = reverse)"
                    : direction > 0 ? "running toward the shooter" : "running reversed");
            telemetry.addData("Target motor rpm", "%.0f", rpm);
            for (Channel c : channels) {
                if (c.motor == null) continue;
                String p = c.label.isEmpty() ? "" : c.label + " ";
                double measured = c.measuredRpm();
                telemetry.addData(p + "measured motor / roller rpm", "%.0f / %.0f", measured, measured / c.ratio);
                telemetry.addData(p + "encoder ticks", c.motor.getCurrentPosition());
                telemetry.addData(p + "current (A)", "%.2f", c.motor.getCurrent(CurrentUnit.AMPS));
                if (!c.fault.isEmpty()) telemetry.addData(p + "FAULT", c.fault);
            }
        }
    }

    /**
     * Both turret servos powered, each on its own stick, to mesh the gears and see whether the servos fight
     * (hub current goes up and they get warm when their positions disagree).
     */
    private class TwoServoStep extends Step {
        final double[] a, b;
        boolean powered;

        TwoServoStep(double[] a, double[] b) {
            super("Turret A and B, separate sticks", new Srv[]{turretA, turretB}, new String[]{
                    "Both gears on. Left stick = servo A, right stick = servo B.",
                    "Dpad left/right = fine A, left/right bumper = fine B. B button = limp.",
                    "If the servos disagree they fight: move one until the hub current is lowest."});
            this.a = a;
            this.b = b;
        }

        @Override
        void enter() {
            powered = false;
        }

        @Override
        void loop(Input in, double dt) {
            boolean moved = in.stick != 0 || in.rightStick != 0 || in.left || in.right || in.lb || in.rb;
            if (moved && !powered) {
                powered = true;
            } else if (powered) {
                a[0] = Range.clip(a[0] + in.stick * JOG_RATE * dt + (in.right ? FINE_STEP : 0)
                        - (in.left ? FINE_STEP : 0), 0, 1);
                b[0] = Range.clip(b[0] + in.rightStick * JOG_RATE * dt + (in.rb ? FINE_STEP : 0)
                        - (in.lb ? FINE_STEP : 0), 0, 1);
            }
            if (powered) {
                turretA.set(a[0]);
                turretB.set(b[0]);
            }
        }

        @Override
        void addTelemetry() {
            telemetry.addData("Now", "%sservo A %.4f, servo B %.4f%s",
                    powered ? "" : "LIMP (push a stick or button to power both here) ", a[0], b[0],
                    feedbackText(turretA));
            telemetry.addData("Hub current", "%.2f A", hubAmps());
        }
    }

    private PositionTestStep dropdownTestStep() {
        return new PositionTestStep("Dropdown up/down test", new Srv[]{dropLeft, dropRight},
                "down (DEPLOYED)", "up (STOWED)",
                down -> {
                    dropLeft.set(down ? DROPDOWN_LEFT_DEPLOYED : DROPDOWN_LEFT_STOWED);
                    dropRight.set(down ? DROPDOWN_RIGHT_DEPLOYED : DROPDOWN_RIGHT_STOWED);
                },
                () -> String.format(Locale.US, "up (STOWED) left/right %.4f / %.4f, down (DEPLOYED) %.4f / %.4f",
                        DROPDOWN_LEFT_STOWED, DROPDOWN_RIGHT_STOWED, DROPDOWN_LEFT_DEPLOYED,
                        DROPDOWN_RIGHT_DEPLOYED));
    }

    /**
     * Moves a mechanism between the two positions currently in the constants file, to check them after they have
     * been pasted in and the app rebuilt. A goes to the first position, Y to the second.
     */
    private class PositionTestStep extends Step {
        final String aLabel, yLabel;
        final Consumer<Boolean> drive;
        final Supplier<String> values;
        boolean powered, atA;

        PositionTestStep(String title, Srv[] uses, String aLabel, String yLabel, Consumer<Boolean> drive,
                         Supplier<String> values) {
            super(title, uses, new String[]{
                    "Uses the positions in IntakeConstants (rebuild and reinstall after changing them).",
                    "A = " + aLabel + ", Y = " + yLabel + "."});
            this.aLabel = aLabel;
            this.yLabel = yLabel;
            this.drive = drive;
            this.values = values;
        }

        @Override
        void enter() {
            powered = false;
        }

        @Override
        void loop(Input in, double dt) {
            if (in.a || in.y) {
                atA = in.a;
                powered = true;
            }
            if (powered) drive.accept(atA);
        }

        @Override
        void addTelemetry() {
            telemetry.addData("Now", powered ? (atA ? aLabel : yLabel)
                    : "LIMP - A = " + aLabel + ", Y = " + yLabel);
            telemetry.addData("Positions", values.get());
        }
    }

    private class SummaryStep extends Step {
        SummaryStep() {
            super("Summary", new Srv[0], new String[]{
                    "Copy these into TurretConstants.java and IntakeConstants.java (* = changed this run).",
                    "Also in /sdcard/FIRST/" + FILE_NAME + " (adb pull it). They only last until the app restarts."});
        }

        @Override
        void loop(Input in, double dt) {}

        @Override
        void addTelemetry() {
            for (Entry entry : entries) {
                telemetry.addData((entry.changed() ? "* " : "  ") + entry.name, entry.value());
            }
        }
    }
}
