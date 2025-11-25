package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

enum startingPoses {
    CURRENT,
    BR,
    BL,
    C
}

public class FollowerPose {
    public static Pose pose = new Pose(0, 0, 0);
    public static final double startingDX = 24;

    public static void setStartingPose (startingPoses poseName) {
        switch (poseName) {
            case CURRENT:
                break;

            case BR:
                pose = new Pose (72 + startingDX, Robot.HL, Math.toRadians(270));
                break;

            case BL:
                pose = new Pose (72 - startingDX, Robot.HL, Math.toRadians(270));
                break;
        }
    }

    public static void setEndingPose (Pose endingPose) {
        pose = endingPose;
    }
}