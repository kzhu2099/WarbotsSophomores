package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class P {
    public static Pose startingPose;

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

    public static final double scoringDX = 12;
    public static final double cycleEndDX = 48 + 24 - 12 - Robot.HL;
    public static final double triggerPoseDX = 55;
    public static final double backScoringY = 18;
    public static final double frontScoringY = 72 + 12; // TODO: find this y

    public static final double rowHP = 12;
    public static final double rowI = 36;
    public static final double rowII = 60;
    public static final double rowIII = 84;
    public static final double rowTrigger = 72;

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
                startingPose = new Pose (72 + 48 - Robot.HW, 144 - Robot.HL, Math.toRadians(90));
                break;

            case FL:
                startingPose = new Pose (72 - 48 + Robot.HW, 144 - Robot.HL, Math.toRadians(90));
                break;
        }
    }

    public static void buildMainPoses () {
        double backAngle = Math.atan((144 - backScoringY) / (144 - (72 + scoringDX))) + Math.toRadians(180);
        double frontAngle = Math.atan((144 - frontScoringY) / (14 - (72 + scoringDX))) + Math.toRadians(180);

        // frontAngle = Math.toRadians(90);
        // backAngle = Math.toRadians(270);
        BR_ScoringPose = new Pose (72 + scoringDX, backScoringY, backAngle);
        FR_ScoringPose = new Pose (72 + scoringDX, frontScoringY, frontAngle);

        backAngle = 0 - Math.atan((144 - backScoringY) / (72 - scoringDX));
        frontAngle = 0 - Math.atan((144 - frontScoringY) / (72 - scoringDX));

        // frontAngle = Math.toRadians(90);
        // backAngle = Math.toRadians(270);
        BL_ScoringPose = new Pose (72 - scoringDX, backScoringY, backAngle);
        FL_ScoringPose = new Pose (72 - scoringDX, frontScoringY, frontAngle);

        BR_EndingPose = new Pose (72 + 36, 12, Math.toRadians(270));
        BL_EndingPose = new Pose (72 - 36, 12, Math.toRadians(270));
        FR_EndingPose = new Pose (72 + 12, 144 - 12, Math.toRadians(60));
        FL_EndingPose = new Pose (72 - 12, 144 - 12, Math.toRadians(180 - 60));

        R_TriggerPose = new Pose (72 + triggerPoseDX, rowTrigger, Math.toRadians(0));
        L_TriggerPose = new Pose (72 - triggerPoseDX, rowTrigger, Math.toRadians(180));

        R_ParkingPose = new Pose (72 + 33, 33, Math.toRadians(270));
        L_ParkingPose = new Pose (72 - 33, 33, Math.toRadians(270));
    }

    public static void buildOtherAutoPaths (@NonNull Follower follower) {
        BR_Preload = follower.pathBuilder()
                .addPath(new BezierLine(P.startingPose, P.BR_ScoringPose))
                .setConstantHeadingInterpolation(P.BR_ScoringPose.getHeading())
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
                // .setTranslationalConstraint(2)
                // .setHeadingConstraint(Math.toRadians(10))
                // .setTimeoutConstraint(0.7)
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