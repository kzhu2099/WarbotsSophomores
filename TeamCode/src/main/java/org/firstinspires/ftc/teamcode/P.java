package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class P {
    public static Pose startingPose = new Pose(0, 0, 0);

    public static Pose BR_ScoringPose;
    public static Pose FR_ScoringPose;
    public static Pose BL_ScoringPose;
    public static Pose FL_ScoringPose;

    public static Pose BR_EndingPose;
    public static Pose FR_EndingPose;
    public static Pose BL_EndingPose;
    public static Pose FL_EndingPose;

    public static Pose R_TriggerPose;
    public static Pose L_TriggerPose;
    public static Pose R_ParkingPose;
    public static Pose L_ParkingPose;

    public static final double scoringDX = 14;
    public static final double cycleEndDX = 50;
    public static final double triggerPoseDX = 55;
    public static final double backScoringY = 15;
    public static final double frontScoringY = 144 - 58; // TODO: find this y

    public static final double rowI = 36;
    public static final double rowII = 60;
    public static final double rowIII = 84;
    public static final double rowTrigger = 72 - 3.5;

    public static PathChain BR_Preload;
    public static PathChain BR_End;

    public static PathChain FR_Preload;
    public static PathChain FR_End;

    public static PathChain BL_Preload;
    public static PathChain BL_End;

    public static PathChain FL_Preload;
    public static PathChain FL_End;

    public static void setStartingPose (startingPoses poseName) {
        switch (poseName) {
            case CURRENT:
                break;

            case BR:
                startingPose = new Pose (72 + 24, Robot.HL, Math.toRadians(270));
                break;

            case BL:
                startingPose = new Pose (72 - 24, Robot.HL, Math.toRadians(270));
                break;

            case FR:
                startingPose = new Pose (72 + 36, 144 - Robot.HL, Math.toRadians(90));
                break;

            case FL:
                startingPose = new Pose (72 - 36, 144 - Robot.HL, Math.toRadians(90));
                break;
        }
    }

    public static void buildMainPoses () {
        double backAngle = Math.atan((138 - backScoringY) / (138 - (72 + scoringDX))) + Math.toRadians(180);
        double frontAngle = Math.atan((138 - frontScoringY) / (138 - (72 + scoringDX))) + Math.toRadians(180);

        BR_ScoringPose = new Pose (72 + scoringDX, backScoringY, backAngle);
        FR_ScoringPose = new Pose (72 + scoringDX, frontScoringY, frontAngle);
        BL_ScoringPose = new Pose (72 - scoringDX, backScoringY, Math.toRadians(180) - backAngle);
        FL_ScoringPose = new Pose (72 - scoringDX, frontScoringY, Math.toRadians(180) - frontAngle);

        BR_EndingPose = new Pose (72 + 40, 40, Math.toRadians(90));
        FR_EndingPose = new Pose (72 - 40, 40, Math.toRadians(90));
        BL_EndingPose = new Pose (72 + 40, 144 - 60, Math.toRadians(60));
        FL_EndingPose = new Pose (72 - 40, 144 - 60, Math.toRadians(180 - 60));

        R_TriggerPose = new Pose (72 + triggerPoseDX, rowTrigger, Math.toRadians(0));
        L_TriggerPose = new Pose (72 - triggerPoseDX, rowTrigger, Math.toRadians(180));

        R_ParkingPose = new Pose (72 + 33, 33, Math.toRadians(90));
        L_ParkingPose = new Pose (72 - 33, 33, Math.toRadians(90));
    }

    public static void buildOtherAutoPaths (Follower follower) {
        BR_Preload = follower.pathBuilder()
                .addPath(new BezierLine(P.startingPose, P.BR_ScoringPose))
                .setLinearHeadingInterpolation(P.startingPose.getHeading(), P.BR_ScoringPose.getHeading())
                .build();

        BR_End = follower.pathBuilder()
                .addPath(new BezierLine(P.startingPose, P.BR_EndingPose))
                .setLinearHeadingInterpolation(P.startingPose.getHeading(), P.BR_EndingPose.getHeading())
                .build();

        FR_Preload = follower.pathBuilder()
                .addPath(new BezierLine(P.startingPose, P.FR_ScoringPose))
                .setLinearHeadingInterpolation(P.startingPose.getHeading(), P.FR_ScoringPose.getHeading())
                .build();

        FR_End = follower.pathBuilder()
                .addPath(new BezierLine(P.startingPose, P.FR_EndingPose))
                .setLinearHeadingInterpolation(P.startingPose.getHeading(), P.FR_EndingPose.getHeading())
                .build();

        BL_Preload = follower.pathBuilder()
                .addPath(new BezierLine(P.startingPose, P.BL_ScoringPose))
                .setLinearHeadingInterpolation(P.startingPose.getHeading(), P.BL_ScoringPose.getHeading())
                .build();

        BL_End = follower.pathBuilder()
                .addPath(new BezierLine(P.startingPose, P.BL_EndingPose))
                .setLinearHeadingInterpolation(P.startingPose.getHeading(), P.BL_EndingPose.getHeading())
                .build();

        FL_Preload = follower.pathBuilder()
                .addPath(new BezierLine(P.startingPose, P.FL_ScoringPose))
                .setLinearHeadingInterpolation(P.startingPose.getHeading(), P.FL_ScoringPose.getHeading())
                .build();

        FL_End = follower.pathBuilder()
                .addPath(new BezierLine(P.startingPose, P.FL_EndingPose))
                .setLinearHeadingInterpolation(P.startingPose.getHeading(), P.FL_EndingPose.getHeading())
                .build();
    }

    public static void setStoppedPose (Pose endingPose) {
        startingPose = endingPose;
    }
}