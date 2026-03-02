package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13) // estimate from Mack: 25 pounds, overestimate is better
            .forwardZeroPowerAcceleration(-35)
            .lateralZeroPowerAcceleration(-60)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.075,
                    0.0001,
                    0.001,
                    0.01
            ))
            /*.translationalPIDFSwitch(6)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0,
                    0,
                    0,
                    0
            ))*/
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.6,
                    0.0001,
                    0.01,
                    0.005
            ))
            /*.headingPIDFSwitch(Math.toRadians(10))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    0.5,
                    0.001,
                    0.175,
                    0
            ))*/
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.01,
                    0.0001,
                    0.0001,
                    0.6,
                    0
            ))
            /*.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0,
                    0,
                    0.00,
                    0.6,
                    0
            ))*/
            //.drivePIDFSwitch(15)
            .centripetalScaling(0.00037)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("fl")
            .leftRearMotorName("bl")
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(50)
            .yVelocity(60);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.75)
            .strafePodX(0.2)
            .hardwareMapName("odom")
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            1,
            1,
            Math.toRadians(1),
            50,
            1.3,
            10,
            0.8
    );

    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}