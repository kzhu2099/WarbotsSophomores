package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.CustomCurve;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

public class Robot {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private DcMotor intakeMotor;
    private DcMotorEx outtakeLeftMotor;
    private DcMotorEx outtakeRightMotor;
    private CRServo backServo;

    private static final double SENSITIVITY = 0.05;
    private static final double SERVOPOWER = 1;
    private static final double INTAKEPOWER = 0.7;
    private static final double FRONTOUTTAKERPM = 3400;
    private static final double BACKOUTTAKERPM = 3600;
    private static final double EMERGENCYOUTTAKERPM = 5500;
    private static final double OUTTAKECPR = 28;
    private static double frontOuttakeAngularRate;
    private static double backOuttakeAngularRate;
    private static double emergencyOuttakeAngularRate;
    private static final double UPTOSPEEDTHRESHOLD = 0.99;
    private Follower follower;
    private static boolean redAlliance;

    public Robot (HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, boolean redAlliance) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.redAlliance = redAlliance;

        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        intakeMotor = hardwareMap.get(DcMotor.class, "in");
        outtakeLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lo");
        outtakeRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ro");

        // frontServo = hardwareMap.get(CRServo.class, "Front Servo");
        backServo = hardwareMap.get(CRServo.class, "bs");
        // odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odom");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeRightMotor.setDirection(DcMotor.Direction.FORWARD);

        backServo.setDirection(CRServo.Direction.REVERSE);

        outtakeLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontOuttakeAngularRate = (FRONTOUTTAKERPM / 60) * OUTTAKECPR;
        backOuttakeAngularRate = (BACKOUTTAKERPM / 60) * OUTTAKECPR;
        emergencyOuttakeAngularRate = (EMERGENCYOUTTAKERPM / 60) * OUTTAKECPR;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private PathChain scorePreload;
    private PathChain autoAimPath;

    private int currentPath;
    private int teleOpState;

    private Pose startingPose;
    private Pose scoringPose;

    public void buildAutoPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, scoringPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), scoringPose.getHeading())
                .build();
    }

    public void teleOpLoop () {
        follower.update();
        if (gamepad1.bWasPressed() && !follower.isBusy()) {
            teleOpState = 1;
            buildAutoAimPath();
            follower.followPath(autoAimPath);
        }

        if (teleOpState == 1) {
            telemetry.addData("autoaim", autoAimPath.getPath(0).getPose(1));
        }

        if (teleOpState == 0) {
            boolean slowMove = gamepad1.right_trigger > SENSITIVITY;
            if (!slowMove) {
                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = -gamepad1.left_stick_x;
                double yaw = -gamepad1.right_stick_x;

                follower.setTeleOpDrive(axial, lateral, yaw, true);
            }

            else {
                double axial = -gamepad1.left_stick_y * 0.5;  // Note: pushing stick forward gives negative value
                double lateral = -gamepad1.left_stick_x * 0.5;
                double yaw = -gamepad1.right_stick_x * 0.5;

                follower.setTeleOpDrive(axial, lateral, yaw, true);
            }

            /*
            boolean slowMove = gamepad1.right_trigger > SENSITIVITY;

            double slowSpeed = slowMove ? 0.5 : 1.0;
            double frontLeftPower = slowSpeed * (axial + lateral + yaw);
            double frontRightPower = slowSpeed * (axial - lateral - yaw);
            double backLeftPower = slowSpeed * (axial - lateral + yaw);
            double backRightPower = slowSpeed * (axial + lateral - yaw);

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            */
        }

        else if (teleOpState == 1) {
            if (!follower.isBusy()) {
                teleOpState = 0;
                follower.startTeleopDrive(true);
            }
        }

        boolean intakeOn = gamepad2.left_trigger > SENSITIVITY;
        boolean outtakeOn = gamepad2.right_trigger > SENSITIVITY;
        boolean backShoot = gamepad2.right_bumper;
        boolean emergencyShoot = gamepad2.left_bumper;

        intakeMotor.setPower(intakeOn ? INTAKEPOWER : 0);

        double angularRate = (backShoot) ? backOuttakeAngularRate : frontOuttakeAngularRate;
        angularRate = (emergencyShoot) ? emergencyOuttakeAngularRate : angularRate;

        outtakeLeftMotor.setVelocity(outtakeOn ? angularRate : 0);
        outtakeRightMotor.setVelocity(outtakeOn ? angularRate : 0);

        boolean outtakeUpToSpeed = checkUpToSpeed(outtakeLeftMotor, angularRate) && checkUpToSpeed(outtakeRightMotor, angularRate);

        boolean backServoOn = gamepad2.y && outtakeUpToSpeed;
        double backServoPower = (backServoOn) ? SERVOPOWER : 0;
        backServo.setPower(backServoPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("intake status", "%b, %4.2f", intakeOn, INTAKEPOWER);
        telemetry.addData("outtake status", "%b, %4.2f", outtakeOn, angularRate);
        telemetry.addData("back servo status", "%b, %4.2f", backServoOn, backServoPower);
        telemetry.addData("driving status", teleOpState);
        telemetry.addData("follower is busy", follower.isBusy());
        odometryTelemetry();

        telemetry.update();
    }

    public void buildAutoAimPath () {
        Pose currentPose = follower.getPose();
        double x = currentPose.getX();
        double y = currentPose.getY();
        double heading;
        Pose targetPose;
        Pose intermediatePose;

        int k = 10;
        if (redAlliance) {
            heading = Math.atan((140 - (y)) / (140 - (x - 9))) + Math.PI; // middle of robot, turn to outtake
            intermediatePose = new Pose(x + k, y + k, heading);
            targetPose = new Pose(x, y, heading);

            autoAimPath = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, intermediatePose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), heading)
                    .addPath(new BezierLine(intermediatePose, targetPose))
                    .setLinearHeadingInterpolation(heading, heading)
                    .build();
        }
    }

    public void bottomRightAutoLoop() {
        follower.update();

        switch (currentPath) {
            case 0:
                follower.followPath(scorePreload);

                if (!follower.isBusy()) {
                    currentPath += 1;
                }

                break;
        }

        odometryTelemetry();
    }

    public void init () {
        follower = Constants.createFollower(hardwareMap);

        if (redAlliance) {
            scoringPose = new Pose (104, 38, Math.toRadians(90));
        }
    }

    public void start () {
        currentPath = 0;
        teleOpState = 0;

        startingPose = FollowerPose.pose;
        follower.setStartingPose(startingPose);

        buildAutoPaths();

        follower.startTeleopDrive(true);
    }

    public void stop () {
        follower.setXVelocity(0);
        follower.setYVelocity(0);

        FollowerPose.setEndingPose(follower.getPose());
    }

    public void delay (double time) {
        double startTime = runtime.seconds();
        while (runtime.seconds() - startTime < time) {
            telemetry.addData("delay", runtime.seconds() - startTime);
        }
    }

    public void setPowers (double[] powers) {
        frontLeftDrive.setPower(powers[0]);
        frontRightDrive.setPower(powers[1]);
        backLeftDrive.setPower(powers[2]);
        backRightDrive.setPower(powers[3]);
    }

    public void zeroPowers() {
        setPowers(new double[] {0, 0, 0, 0});
    }

    public void setIntake (boolean status) {
        intakeMotor.setPower(status ? INTAKEPOWER : 0);
    }

    public void setOuttake (boolean status, boolean backShoot) {
        double angularRate = (backShoot) ? backOuttakeAngularRate : frontOuttakeAngularRate;
        outtakeLeftMotor.setVelocity(status ? angularRate : 0);
        outtakeRightMotor.setVelocity(status ? angularRate : 0);
    }

    public boolean checkUpToSpeed (DcMotorEx motor, double targetSpeed) {
        return motor.getVelocity() >= UPTOSPEEDTHRESHOLD * targetSpeed;
    }

    public void odometryTelemetry () {
        telemetry.addData("position x", follower.getPose().getX());
        telemetry.addData("position y", follower.getPose().getY());
        telemetry.addData("position rotation", Math.toDegrees(follower.getHeading()));
    }
}
