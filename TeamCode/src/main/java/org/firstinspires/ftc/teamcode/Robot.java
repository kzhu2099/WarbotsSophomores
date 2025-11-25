package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Robot {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    /*private final DcMotor frontLeftDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backRightDrive;*/
    private final DcMotor intakeMotor;
    private final DcMotorEx outtakeLeftMotor;
    private final DcMotorEx outtakeRightMotor;
    private final CRServo backServo;

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
    private static final double UPTOSPEEDTHRESHOLD = 1; // measuring velocity is not always accurate so this will be triggered

    public static final double WIDTH = 17.5;
    public static final double LENGTH = 17;
    public static final double HW = WIDTH / 2;
    public static final double HL = LENGTH / 2;

    private static double ERRORCOUNT = 0;

    private Follower follower;
    private static boolean redAlliance;
    private autoCycles[] autoCycleList;

    public Robot (HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, boolean _redAlliance) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        redAlliance = _redAlliance;

        /*frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");*/

        intakeMotor = hardwareMap.get(DcMotor.class, "in");
        outtakeLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lo");
        outtakeRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ro");

        // frontServo = hardwareMap.get(CRServo.class, "Front Servo");
        backServo = hardwareMap.get(CRServo.class, "bs");
        // odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odom");

        /*frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);*/

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

    public void setAutoCycleList(autoCycles[] autoCycleList) {
        this.autoCycleList = autoCycleList;
    }

    private PathChain autoAimPath;

    public void teleOpLoop () {
        follower.update();
        if (gamepad1.bWasPressed() && !follower.isBusy()) {
            teleOpState = 1;
            buildAutoAimPath();
            follower.followPath(autoAimPath);
        }

        if (autoAimPath != null) {
            telemetry.addData("autoaim", Math.toDegrees(autoAimPath.getPath(1).getPose(1).getHeading()));
        }

        if (teleOpState == 0) {
            boolean slowMove = gamepad1.right_trigger > SENSITIVITY;
            double axial;
            double lateral;
            double yaw;

            if (!slowMove) {
                axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                lateral = -gamepad1.left_stick_x;
                yaw = -gamepad1.right_stick_x;
            }

            else {
                axial = -gamepad1.left_stick_y * 0.5;  // Note: pushing stick forward gives negative value
                lateral = -gamepad1.left_stick_x * 0.5;
                yaw = -gamepad1.right_stick_x * 0.5;
            }

            axial = redAlliance ? axial : -axial;
            lateral = redAlliance ? lateral : -lateral;
            follower.setTeleOpDrive(axial, lateral, yaw, false);

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

        telemetry.addData("intake status", "%b, %4.2f", intakeOn, INTAKEPOWER);
        telemetry.addData("outtake status", "%b, %4.2f", outtakeOn, angularRate);
        telemetry.addData("back servo status", "%b, %4.2f", backServoOn, backServoPower);
        telemetry.addData("driving status", teleOpState);
        telemetry.addData("follower is busy", follower.isBusy());

        allTelemetry();
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
            heading = Math.atan((140 - (y)) / (140 - (x - HW))) + Math.PI; // middle of robot, turn to outtake
            intermediatePose = new Pose(x + k, y + k, heading / 2);
            targetPose = new Pose(x, y, heading);

            autoAimPath = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, intermediatePose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), intermediatePose.getHeading())
                    .addPath(new BezierLine(intermediatePose, targetPose))
                    .setLinearHeadingInterpolation(intermediatePose.getHeading(), heading)
                    .build();
        }
    }

    private static final int shootingTime = 4000;
    private static final int triggerTime = 1000;
    private int triggerCycleNumber;
    private int cycleNumber;
    private int teleOpState;
    private int autoCycleState;

    public boolean triggerTrigger;

    private Pose startingPose;
    private Pose backRightScoringPose;
    private Pose frontRightScoringPose;
    private Pose backLeftScoringPose;
    private Pose frontLeftScoringPose;
    private Pose rightTriggerPose;
    private Pose leftTriggerPose;

    private static final double scoringDX = 14;
    private static final double cycleEndDX = 30;
    private static final double bottomCycleEndDXCorrection = 0;
    private static final double backScoringY = 15;
    private static final double frontScoringY = 100; // this has not been tested yet, TODO!

    private static final double rowI = 36;
    private static final double rowII = 60;
    private static final double rowIII = 84;
    private static final double rowTrigger = 72;

    private PathChain BRPreloadGo;

    public void buildScoringPoses () {
        double backAngle = Math.atan((140 - backScoringY) / (68 - scoringDX)) + Math.toRadians(180);

        backRightScoringPose = new Pose (72 + scoringDX, backScoringY, backAngle);
        frontRightScoringPose = new Pose (72 + scoringDX, 100, Math.toRadians(90));
        backLeftScoringPose = new Pose (72 - scoringDX, 38, Math.toRadians(180) - backAngle);
        frontLeftScoringPose = new Pose (72 - scoringDX, 100, Math.toRadians(90));

        rightTriggerPose = new Pose (72 + cycleEndDX, rowTrigger, Math.toRadians(90));
        leftTriggerPose = new Pose (72 - cycleEndDX, rowTrigger, Math.toRadians(90));
    }

    public void buildOtherAutoPaths () {
        BRPreloadGo = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, backRightScoringPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), backRightScoringPose.getHeading())
                .build();

        // TODO: LITERALLY FIX EVERYTHING MAKE SURE IT WORKS THEN REPEAT FOR NEXT 3 THEN NEW BOT GNG
    }

    public PathChain[] buildAutoCyclePaths (Pose start, Pose end) {
        Pose control = new Pose(start.getX(), end.getY());
        PathChain path1 = follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();

        PathChain path2 = follower.pathBuilder()
                .addPath(new BezierCurve(end, control, start))
                .setLinearHeadingInterpolation(end.getHeading(), start.getHeading())
                .build();

        return new PathChain[] {path1, path2};
    }

    public void autoLoop () {
        follower.update();

        autoCycles currentCycle;

        if (cycleNumber < autoCycleList.length) {
            currentCycle = autoCycleList[cycleNumber];
        }

        else {
            currentCycle = autoCycles.NONE;
        }

        if (!follower.isBusy() || autoCycleState == 0) { // won't change if it is busy, the == 0 shouldn't happen
            PathChain[] paths;
            switch (currentCycle) {
                case NONE:
                    break;

                case BR_PRELOAD:
                    autoCycle(BRPreloadGo, null, true, true);
                    break;

                case BR_I:
                    paths = buildAutoCyclePaths(backRightScoringPose, new Pose(72 + cycleEndDX + bottomCycleEndDXCorrection, rowI, 0));
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BR_II:
                    paths = buildAutoCyclePaths(backRightScoringPose, new Pose(72 + cycleEndDX + bottomCycleEndDXCorrection, rowII, 0));
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BR_III:
                    paths = buildAutoCyclePaths(backRightScoringPose, new Pose(72 + cycleEndDX, rowIII, 0));
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BR_TRIGGER:
                    if (triggerTrigger) {
                        paths = buildAutoCyclePaths(backRightScoringPose, rightTriggerPose);
                        autoCycle(paths[0], paths[1], false, true);
                        break;
                    }

                    else {
                        cycleNumber++;
                    }
            }
        }

        allTelemetry();

        telemetry.addData("auto number", cycleNumber);
        telemetry.addData("cycle state", autoCycleState);
        telemetry.addData("cycle", currentCycle);
    }

    public void autoCycle (PathChain go, PathChain back, boolean useBalls, boolean backShootPower) {
        switch (autoCycleState) {
            case -1:
                break;

            case 0:
                if (go != null) {
                    follower.followPath(go);

                    if (useBalls) {
                        setIntake(true);
                    }
                }

                if (triggerCycleNumber != -1 && triggerCycleNumber == cycleNumber) {
                    autoCycleState = 10;
                }

                else {
                    autoCycleState = 1;
                }

            case 1:
                if (!follower.isBusy()) {
                    if (useBalls) {
                        setIntake(false);
                        setOuttake(true, backShootPower);
                    }

                    if (back != null) {
                        follower.followPath(back);
                    }

                    autoCycleState = 2;
                }

            case 2:
                if (!useBalls) {
                    autoCycleState = 0;
                    cycleNumber++;
                }

                else if (checkUpToSpeed(outtakeLeftMotor, (backShootPower) ? backOuttakeAngularRate : frontOuttakeAngularRate)
                                && !follower.isBusy()) {

                    autoCycleState = -1; // prevent 2 again but also not 0

                    Thread thread = new Thread(() -> {
                        setBackServo(true);
                        setIntake(true);

                        try {
                            Thread.sleep(shootingTime);
                        } catch (Exception e) {
                            ERRORCOUNT++;
                        }

                        setBackServo(false);
                        setIntake(false);
                        setOuttake(false, backShootPower);

                        autoCycleState = 0;
                        cycleNumber++;
                    });

                    thread.start();
                }

            case 10:
                if (!follower.isBusy()) {
                    Pose currentPose = follower.getPose();
                    Pose triggerPose = (redAlliance) ? rightTriggerPose : leftTriggerPose;

                    PathChain goTrigger = follower.pathBuilder()
                            .addPath(new BezierLine(currentPose, triggerPose))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), triggerPose.getHeading())
                            .build();

                    follower.followPath(goTrigger);
                    autoCycleState = 11;
                }

            case 11:
                if (!follower.isBusy()) {
                    Pose currentPose = follower.getPose();
                    Pose triggerPose = (redAlliance) ? rightTriggerPose : leftTriggerPose;

                    PathChain comeBack = follower.pathBuilder() // go prepare for the back path
                            .addPath(new BezierLine(currentPose, triggerPose))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), triggerPose.getHeading())
                            .build();

                    Thread thread = new Thread(() -> {
                        autoCycleState = -1;

                        try {
                            Thread.sleep(triggerTime);
                        } catch (Exception e) {
                            ERRORCOUNT++;
                        }

                        follower.followPath(comeBack);

                        autoCycleState = 1;
                    });

                    thread.start();
                }
        }
    }

    public void init () {
        follower = Constants.createFollower(hardwareMap);
    }

    private static final startingPoses[] allStartingPoses = startingPoses.values();
    private static int startingSelection = 0;

    public void teleOpInitLoop () {
        if (gamepad1.dpadRightWasPressed()) {
            startingSelection = (startingSelection + 1) % allStartingPoses.length;
        }

        else if (gamepad1.dpadLeftWasPressed()) {
            startingSelection = (startingSelection - 1 + allStartingPoses.length) % allStartingPoses.length;
        }

        telemetry.addData("starting pose", allStartingPoses[startingSelection]);
        allTelemetry();

        telemetry.update();
    }

    public void autoInitLoop () {
        if (gamepad1.leftBumperWasPressed()) {
            triggerTrigger = false;
        }

        else if (gamepad1.rightBumperWasPressed()) {
            triggerTrigger = true;
        }

        if (gamepad2.leftBumperWasPressed()) { // init is where the cycle list is called
            triggerCycleNumber = Math.max(-1, triggerCycleNumber - 1);
        }

        else if (gamepad2.rightBumperWasPressed()) {
            triggerCycleNumber = Math.min(autoCycleList.length - 1, triggerCycleNumber + 1);
        }

        telemetry.addData("triggering reset for balls", triggerTrigger);

        autoCycles cycle;
        if (triggerCycleNumber == -1) {
            cycle = autoCycles.NONE;
        }
        else {
            cycle = autoCycleList[triggerCycleNumber];
        }
        telemetry.addData("the cycle during which the trigger will be triggered", cycle);
        allTelemetry();

        telemetry.update();
    }

    public void start () {
        cycleNumber = 0;
        teleOpState = 0;

        startingPose = FollowerPose.pose;
        follower.setStartingPose(startingPose);

        buildScoringPoses();
        buildOtherAutoPaths();

        follower.startTeleopDrive(true);
    }

    public void stop () {
        follower.setXVelocity(0);
        follower.setYVelocity(0);

        FollowerPose.setEndingPose(follower.getPose());
    }

    public void setIntake (boolean status) {
        intakeMotor.setPower(status ? INTAKEPOWER : 0);
    }

    public void setOuttake (boolean status, boolean backShoot) {
        double angularRate = (backShoot) ? backOuttakeAngularRate : frontOuttakeAngularRate;
        outtakeLeftMotor.setVelocity(status ? angularRate : 0);
        outtakeRightMotor.setVelocity(status ? angularRate : 0);
    }

    public void setBackServo (boolean status) {
        backServo.setPower(status ? 1 : 0);
    }

    public boolean checkUpToSpeed (DcMotorEx motor, double targetSpeed) {
        return motor.getVelocity() >= UPTOSPEEDTHRESHOLD * targetSpeed;
    }

    public void allTelemetry () {
        telemetry.addData("position x", follower.getPose().getX());
        telemetry.addData("position y", follower.getPose().getY());
        telemetry.addData("position rotation", Math.toDegrees(follower.getHeading()));
        telemetry.addData("errors caught", ERRORCOUNT);
    }
}