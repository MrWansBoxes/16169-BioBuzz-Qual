package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

/**
 * Ring buffer of (time, robot pose, turret angle, camera pitch) samples. Camera frames are ~50-100 ms old by the
 * time we process them, so a detection has to be combined with where the robot, turret and camera tilt were when
 * the frame was captured, not where they are now.
 */
class PoseHistory {
    static final class Sample {
        final long nanos;
        final Pose pose;
        final double turretRad;
        final double cameraPitchRad;

        Sample(long nanos, Pose pose, double turretRad, double cameraPitchRad) {
            this.nanos = nanos;
            this.pose = pose;
            this.turretRad = turretRad;
            this.cameraPitchRad = cameraPitchRad;
        }
    }

    private final Sample[] buffer;
    private int head = -1;
    private int size = 0;

    PoseHistory(int capacity) {
        buffer = new Sample[capacity];
    }

    void add(long nanos, Pose pose, double turretRad, double cameraPitchRad) {
        head = (head + 1) % buffer.length;
        buffer[head] = new Sample(nanos, pose, turretRad, cameraPitchRad);
        size = Math.min(size + 1, buffer.length);
    }

    void clear() {
        head = -1;
        size = 0;
    }

    Sample latest() {
        return size == 0 ? null : buffer[head];
    }

    /** Interpolated sample at {@code nanos}, clamped to the oldest/newest stored sample. */
    Sample at(long nanos) {
        if (size == 0) return null;
        Sample newer = buffer[head];
        if (nanos >= newer.nanos) return newer;
        for (int i = 1; i < size; i++) {
            Sample older = buffer[(head - i + buffer.length) % buffer.length];
            if (older.nanos <= nanos) {
                if (newer.nanos == older.nanos) return newer;
                double t = (double) (nanos - older.nanos) / (newer.nanos - older.nanos);
                return interpolate(older, newer, t, nanos);
            }
            newer = older;
        }
        return newer;
    }

    /** Field-frame velocity (in/s, in/s, rad/s) averaged over roughly the last {@code windowSec}. */
    double[] velocity(double windowSec) {
        Sample now = latest();
        if (now == null) return new double[]{0, 0, 0};
        Sample then = at(now.nanos - (long) (windowSec * 1e9));
        double dt = (now.nanos - then.nanos) / 1e9;
        if (dt < 1e-3) return new double[]{0, 0, 0};
        return new double[]{
                (now.pose.x() - then.pose.x()) / dt,
                (now.pose.y() - then.pose.y()) / dt,
                Angle.normalizeSigned(now.pose.heading() - then.pose.heading()) / dt
        };
    }

    private static Sample interpolate(Sample a, Sample b, double t, long nanos) {
        double heading = a.pose.heading() + Angle.normalizeSigned(b.pose.heading() - a.pose.heading()) * t;
        Pose pose = new Pose(
                a.pose.x() + (b.pose.x() - a.pose.x()) * t,
                a.pose.y() + (b.pose.y() - a.pose.y()) * t,
                heading);
        return new Sample(nanos, pose, a.turretRad + (b.turretRad - a.turretRad) * t,
                a.cameraPitchRad + (b.cameraPitchRad - a.cameraPitchRad) * t);
    }
}
