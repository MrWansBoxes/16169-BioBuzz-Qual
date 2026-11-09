package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

/**
 * An Axon Mini in servo (positional) mode, every servo on this robot. Sets the PWM range to Axon's 500-2500 us,
 * so 0..1 covers the servo's full programmed travel ({@link #RANGE_DEG}) instead of the SDK default 600-2400 us.
 *
 * Axon Mini+ servos also have a 4th wire with an analog position signal. Where it is plugged into an analog
 * port (turret and camera tilt), {@link #measuredPosition} turns that voltage back into a 0..1 servo position.
 */
public class AxonServo {
    /** Axon default input range. If you reprogram the servos with the Axon programmer, change these to match. */
    public static int PWM_MIN_US = 500;
    public static int PWM_MAX_US = 2500;
    /** Degrees of servo travel from position 0 to 1 (Axon default 355). */
    public static double RANGE_DEG = 355.0;

    private final ServoImplEx servo;
    private final AnalogInput feedback;

    /** @param feedbackName analog port of the feedback wire, or "" if it is not wired. */
    public AxonServo(HardwareMap hardwareMap, String servoName, String feedbackName) {
        servo = hardwareMap.get(ServoImplEx.class, servoName);
        servo.setPwmRange(new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));
        feedback = feedbackName.isEmpty() ? null : hardwareMap.tryGet(AnalogInput.class, feedbackName);
    }

    public void setPosition(double position) {
        servo.setPosition(Range.clip(position, 0, 1));
    }

    public boolean hasFeedback() {
        return feedback != null;
    }

    /** Raw feedback voltage, or NaN without feedback. Shown in telemetry to calibrate {@link #measuredPosition}. */
    public double feedbackVolts() {
        return feedback == null ? Double.NaN : feedback.getVoltage();
    }

    /**
     * Servo position (0..1) the feedback wire reports, or NaN without feedback. Calibrate per servo: command
     * position 0 and position 1 (or two positions close to them and extrapolate) and record the voltages.
     */
    public double measuredPosition(double voltsAtZero, double voltsAtOne) {
        if (feedback == null || voltsAtOne == voltsAtZero) return Double.NaN;
        return (feedback.getVoltage() - voltsAtZero) / (voltsAtOne - voltsAtZero);
    }
}
