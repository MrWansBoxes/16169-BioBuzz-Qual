package org.firstinspires.ftc.teamcode.subsystems.intake;

/**
 * Tunables for the intake (2 motors), the gate servo that holds balls back from the flywheels, and the dropdown
 * that reaches POLLEN on the FLOWERS.
 *
 * Ball path (CAD): front funnels -> roller bed (4 shafts of silicone wheels / surgical tubing, motor A) ->
 * counter-rotating transfer pair at the back (motor B) -> gate -> up through the turntable into the flywheels.
 * Two motors so that a second ball loading into the roller bed can't stall the transfer and drop its RPM.
 */
public final class IntakeConstants {
    private IntakeConstants() {}

    // ---------------- Intake motors (goBILDA bare motors) ----------------
    /** Roller bed. CAD: 24T -> 64T GT2 (2.67:1), then 32T:32T belts to the other shafts. */
    public static String MOTOR_A_NAME = "intakeFront";
    /** Transfer pair. CAD: 24T -> 62T GT2 (2.58:1), shafts geared 48T:48T so they counter-rotate. */
    public static String MOTOR_B_NAME = "intakeTransfer";
    public static boolean MOTOR_A_REVERSED = false;
    public static boolean MOTOR_B_REVERSED = false;

    // Power per mode for each motor (positive = toward the shooter). The transfer runs flat out while intaking
    // and balls stack up against the closed gate.
    public static double INTAKE_POWER_A = 1.0;
    public static double INTAKE_POWER_B = 1.0;
    public static double FEED_POWER_A = 1.0;
    public static double FEED_POWER_B = 1.0;
    public static double EJECT_POWER_A = -0.8;
    public static double EJECT_POWER_B = -0.8;

    // ---------------- Jam handling (motor A) ----------------
    /** Current that means the roller is stalled. 0 disables jam detection (current reads cost loop time). */
    public static double JAM_CURRENT_AMPS = 0.0;
    /** Current must stay above the limit this long before we call it a jam (ignores the spike when starting). */
    public static double JAM_TIME_SEC = 0.25;
    /** How long to run in reverse to clear a jam before intaking again. */
    public static double UNJAM_TIME_SEC = 0.2;

    // ---------------- Gate servo (Axon Mini on the top plate, not the turret) ----------------
    public static String GATE_SERVO_NAME = "gate";
    public static double GATE_CLOSED = 0.2;
    public static double GATE_OPEN = 0.6;
    /** Time for the gate to fully open or close. Measure it; feeding waits this long before pushing balls. */
    public static double GATE_MOVE_SEC = 0.12;

    // ---------------- Feeding ----------------
    /** Gate-open time that lets exactly one ball through. Tune with a full hopper. */
    public static double ONE_BALL_FEED_SEC = 0.15;
    /** How long to feed for "dump everything" shots (no flywheel recovery between balls). */
    public static double FEED_ALL_SEC = 1.0;

    // ---------------- Dropdown (two Axon Minis winding string spools; design not final) ----------------
    // Lowers an intake to grab POLLEN from the FLOWERS. Must start STOWED (inside the 18 in. starting size).
    // The two spools are mirrored, so each servo has its own positions.
    public static String DROPDOWN_LEFT_NAME = "dropdownLeft";
    public static String DROPDOWN_RIGHT_NAME = "dropdownRight";
    public static double DROPDOWN_LEFT_STOWED = 0.2;
    public static double DROPDOWN_LEFT_DEPLOYED = 0.8;
    public static double DROPDOWN_RIGHT_STOWED = 0.8;
    public static double DROPDOWN_RIGHT_DEPLOYED = 0.2;
    /** Time to fully lower or raise. Measure it. */
    public static double DROPDOWN_MOVE_SEC = 0.6;
}
