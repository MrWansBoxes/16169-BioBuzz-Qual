package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;

import java.util.Map;

/**
 * Wraps the real localizer (Pinpoint) and adds a field-frame (x, y) offset to the pose it reports, so the turret
 * camera can fix odometry drift ({@code Turret.correctPose}). Heading and velocities pass through unchanged.
 *
 * The offset is kept in software instead of calling the Pinpoint's setPose: that is an I2C write each time, and it
 * throws away whatever the robot moved between the last read and the write.
 *
 * {@link #setPose} (start of auto / TeleOp) clears the offset.
 */
public class CorrectedLocalizer implements Localizer {
    private final Localizer inner;
    private double offsetX = 0, offsetY = 0;

    public CorrectedLocalizer(Localizer inner) {
        this.inner = inner;
    }

    /** Move the reported pose by (dx, dy) inches on the field. */
    public void shift(double dx, double dy) {
        offsetX += dx;
        offsetY += dy;
    }

    /** Total camera shift since the last {@link #setPose}, {x, y} inches. */
    public double[] offset() {
        return new double[]{offsetX, offsetY};
    }

    @Override
    public void setPose(Pose pose) {
        inner.setPose(pose);
        offsetX = 0;
        offsetY = 0;
    }

    @Override
    public MotionState state() {
        MotionState state = inner.state();
        if (state == null || (offsetX == 0 && offsetY == 0)) return state;
        Pose p = state.pose();
        return state.withPose(new Pose(p.x() + offsetX, p.y() + offsetY, p.heading()));
    }

    @Override
    public void update() {
        inner.update();
    }

    @Override
    public void reset() {
        inner.reset();
        offsetX = 0;
        offsetY = 0;
    }

    @Override
    public Map<String, Object> debug() {
        return inner.debug();
    }
}
