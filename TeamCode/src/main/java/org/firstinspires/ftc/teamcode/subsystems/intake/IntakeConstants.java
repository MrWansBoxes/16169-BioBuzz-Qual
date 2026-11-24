package org.firstinspires.ftc.teamcode.subsystems.intake;

/**
 * Tunables for the intake (2 motors) and the dropdown that reaches POLLEN on the FLOWERS.
 *
 * Ball path (CAD): front funnels -> roller bed (4 shafts of silicone wheels / surgical tubing, motor A) ->
 * counter-rotating transfer pair at the back (motor B) -> up through the turntable into the flywheels.
 * There is no gate: the transfer (B) stays off while intaking, so balls wait in it, and only runs to shoot.
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

    // Speed per mode for each motor, in RPM at the MOTOR shaft (the encoder is on the motor; positive = toward
    // the shooter). Both motors run on the hub's velocity loop (setVelocity), so the speed doesn't change with
    // battery voltage. That matters for B: it launches the balls into the flywheels, so its speed is part of
    // every shot. Tune the flywheel distance table with FEED_RPM_B at its final value.
    // The transfer (B) must stay at 0 while intaking: it is what holds the balls back from the flywheels.
    // goBILDA bare motor free speed is about 6000 rpm at 12 V; stay well under it so the loop has headroom
    // when the battery sags, or the speed stops being regulated.
    public static double INTAKE_RPM_A = 4500.0;
    public static double INTAKE_RPM_B = 0.0;
    public static double FEED_RPM_A = 4500.0;
    public static double FEED_RPM_B = 4500.0;
    public static double EJECT_RPM_A = -3600.0;
    public static double EJECT_RPM_B = -3600.0;

    /** goBILDA bare motor encoder: 28 ticks per motor revolution. */
    public static double TICKS_PER_REV = 28.0;
    /**
     * If a motor's encoder reads under this share of its target after {@link #ENCODER_CHECK_SEC} of running, the
     * encoder is treated as unplugged or counting backwards, and that motor drops to plain power
     * (target / {@link #OPEN_LOOP_FULL_RPM}) for the rest of the OpMode. Without this, the velocity loop would
     * push it to full power.
     */
    public static double ENCODER_FAIL_FRACTION = 0.05;
    public static double ENCODER_CHECK_SEC = 0.5;
    /** RPM that full power gives, only used for the plain-power fallback. */
    public static double OPEN_LOOP_FULL_RPM = 6000.0;

    // ---------------- Jam handling (motor A) ----------------
    /** Current that means the roller is stalled. 0 disables jam detection (current reads cost loop time). */
    public static double JAM_CURRENT_AMPS = 0.0;
    /** Current must stay above the limit this long before we call it a jam (ignores the spike when starting). */
    public static double JAM_TIME_SEC = 0.25;
    /** How long to run in reverse to clear a jam before intaking again. */
    public static double UNJAM_TIME_SEC = 0.2;

    // ---------------- Feeding ----------------
    /**
     * How long both motors run for a shot. Every ball goes back to back (no flywheel recovery between balls).
     * Make it long enough that a full load always clears, with a little margin.
     */
    public static double FEED_ALL_SEC = 1.0;

    // ---------------- Dropdown (two Axon Minis at the front corners driving the bars directly) ----------------
    // Lowers an intake to grab POLLEN from the FLOWERS. Must start STOWED (inside the 18 in. starting size).
    // The two servos face each other (mirrored), so each has its own positions.
    public static String DROPDOWN_LEFT_NAME = "dropdownLeft";
    public static String DROPDOWN_RIGHT_NAME = "dropdownRight";
    public static double DROPDOWN_LEFT_STOWED = 0.87;
    public static double DROPDOWN_LEFT_DEPLOYED = 0.46;
    public static double DROPDOWN_RIGHT_STOWED = 0.485;
    public static double DROPDOWN_RIGHT_DEPLOYED = 0.897;
    /** Time to fully lower or raise. Measure it. */
    public static double DROPDOWN_MOVE_SEC = 0.3;
}
