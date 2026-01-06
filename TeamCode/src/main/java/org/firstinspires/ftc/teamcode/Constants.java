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
            .lateralZeroPowerAcceleration(-65)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    1,
                    0.1,
                    0.1,
                    0
            ))
            .translationalPIDFSwitch(6)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.05,
                    0.0001,
                    0.02,
                    0
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.5,
                    0.003,
                    0.07,
                    0
            ))
            .headingPIDFSwitch(Math.toRadians(10))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    1,
                    0.001,
                    0.175,
                    0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.4,
                    0.0001,
                    0.001,
                    0.6,
                    0
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.05,
                    0.0001,
                    0.0001,
                    0.6,
                    0
            ))
            .drivePIDFSwitch(6)
            .centripetalScaling(0.00055);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("fl")
            .leftRearMotorName("bl")
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(70)
            .yVelocity(50);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3)
            .strafePodX(-5.75)
            .hardwareMapName("odom")
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.25,
            0.25,
            Math.toRadians(0.5),
            50,
            1.1,
            10,
            8
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