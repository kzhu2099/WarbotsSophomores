package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

enum Poses {
    CURRENT,
    BR,
    B,
    C
}
public class FollowerPose {
    public static Pose pose = new Pose(0, 0, 0);

    public static void setStartingPose (Poses poseName) {
        switch (poseName) {
            case CURRENT:
                break;

            case BR:
                pose = new Pose (104, 18, Math.toRadians(90));
                break;
        }
    }

    public static void setEndingPose (Pose endingPose) {
        pose = endingPose;
    }
}