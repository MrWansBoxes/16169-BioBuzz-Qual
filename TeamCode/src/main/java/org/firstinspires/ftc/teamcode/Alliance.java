package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public enum Alliance {
    RED("RED"),
    BLUE("BLUE");

    /** Prefix of this alliance's AprilTag cluster names in the BioBuzz tag library ("RED SCORING", "RED AUDIENCE", ...). */
    public final String clusterPrefix;

    Alliance(String clusterPrefix) {
        this.clusterPrefix = clusterPrefix;
    }

    public boolean ownsCluster(String clusterName) {
        return clusterName != null && clusterName.startsWith(clusterPrefix);
    }

    /**
     * Convert a pose written for the red alliance into this alliance's equivalent. The BIOBUZZ field is
     * 180-degree rotationally symmetric about its center (red/blue HIVEs, LOADING ZONES, GARDENS and FLOWERS
     * all map onto each other), so blue = red rotated half a turn around (72, 72).
     */
    public Pose fromRed(Pose redPose) {
        if (this == RED) return redPose;
        return new Pose(144 - redPose.x(), 144 - redPose.y(), redPose.heading() + Math.PI);
    }
}
