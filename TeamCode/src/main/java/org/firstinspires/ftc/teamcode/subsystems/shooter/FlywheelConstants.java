package org.firstinspires.ftc.teamcode.subsystems.shooter;

/**
 * Two counter-rotating flywheels on the turret with the ball pinched between them. Each is driven 1:1 (20T:20T
 * GT2 belt) by its own goBILDA bare motor.
 * Exit speed ~ the average of the two wheel surface speeds; spin ~ their difference. Equal speeds = no spin.
 */
public final class FlywheelConstants {
    private FlywheelConstants() {}

    public static String MOTOR_A_NAME = "flywheelA";
    public static String MOTOR_B_NAME = "flywheelB";
    /**
     * The wheels turn opposite ways, so usually one motor is reversed. With both flags right, a positive target
     * makes BOTH wheels push the ball out. Check at low speed with no ball first.
     */
    public static boolean MOTOR_A_REVERSED = false;
    public static boolean MOTOR_B_REVERSED = true;

    /** Encoder ticks per wheel revolution (goBILDA 5203 1:1 = 28; multiply by any gear ratio). */
    public static double TICKS_PER_REV = 28.0;

    // ---------------- Velocity control (power = kS + kV * target + kP * error, scaled to 12 V) ----------------
    // Tune kV first: with kP = 0, set kV so the measured RPM matches the target, then add kP for recovery
    // after each shot. Tune both wheels; they are rarely identical.
    public static double A_KS = 0.0;
    public static double A_KV = 1.0 / 6000.0;
    public static double A_KP = 0.0005;
    public static double B_KS = 0.0;
    public static double B_KV = 1.0 / 6000.0;
    public static double B_KP = 0.0005;
    public static double NOMINAL_VOLTAGE = 12.0;
    /**
     * Below target by more than this, ignore the formula and apply full power (spin-up and recovery after a
     * shot). Inside this band the feedforward + P controller settles smoothly without overshoot.
     * Too small = chatter around the setpoint; too large = slower recovery.
     */
    public static double FULL_POWER_BELOW_RPM = 250.0;

    /**
     * Residual spin trim. Wheel A runs at target * (1 + SPIN_BIAS), wheel B at target * (1 - SPIN_BIAS).
     * 0 = no spin. Nudge it if shots consistently sail high/drop (backspin/topspin left over).
     */
    public static double SPIN_BIAS = 0.0;

    /** Both wheels must be within this of their target... */
    public static double READY_TOLERANCE_RPM = 75.0;
    /** ...for this long before {@code isReady()} says go. */
    public static double READY_TIME_SEC = 0.1;

    // ---------------- Distance -> RPM table (measure on the field; placeholders) ----------------
    // Distance = horizontal inches from turret pivot to the raised CELL opening (Turret.targetDistance()).
    // Must be sorted by distance. Outside the table the end values are used.
    public static double[] TABLE_DISTANCE_IN = {24, 48, 72, 96, 120};
    public static double[] TABLE_RPM = {2800, 3200, 3700, 4200, 4700};

    /** Speed to idle at between shots so recovery is faster (0 = coast). */
    public static double IDLE_RPM = 0.0;
}
