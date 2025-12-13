package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierPoint;
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
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    private final DcMotor intakeMotor;
    private final DcMotorEx outtakeLeftMotor;
    private final DcMotorEx outtakeRightMotor;
    private final CRServo backServo;
    private final Servo gateServo;

    private static final double gateOffPosition = 0.25;
    private static final double gateOnPosition = 0;

    private static final double SENSITIVITY = 0.1;
    private static final double SERVOPOWER = 1;
    private static final double INTAKEPOWER = 0.5;
    private static final double SLOWINTAKEPOWER = 0.3;
    private static final double FRONTOUTTAKERPM = 3450;
    private static final double BACKOUTTAKERPM = 3575;
    private static final double EMERGENCYOUTTAKERPM = 5500;
    private static final double OUTTAKECPR = 28;
    private static final double frontOuttakeAngularRate = toAngularRate(FRONTOUTTAKERPM, OUTTAKECPR);
    private static final double backOuttakeAngularRate = toAngularRate(BACKOUTTAKERPM, OUTTAKECPR);
    private static final double emergencyOuttakeAngularRate = toAngularRate(EMERGENCYOUTTAKERPM, OUTTAKECPR);
    private static final double UPTOSPEEDTHRESHOLD = 0.98; // measuring velocity is not always accurate so this will be triggered

    public static final double WIDTH = 17.5;
    public static final double LENGTH = 17;
    public static final double HW = WIDTH / 2;
    public static final double HL = LENGTH / 2;

    private static double SLEEPERRORS = 0;

    private Follower follower;
    private static boolean redAlliance;

    private autoCycles[] autoCycleList;

    private static final int shootingTime = 2500;
    private static final int gateWaitTime = 500;
    private static final int autoIntakeWaitTime = 500;
    private static final int triggerTime = 500;
    private static final int intakeWaitTime = 500;
    private static int triggerCycleNumber = -1;

    private int cycleNumber = 0;
    private int teleOpState = 0;
    private int autoCycleState = 0;

    private boolean autoAimed = false;

    public Robot (HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, boolean _redAlliance) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        redAlliance = _redAlliance;
        intakeMotor = hardwareMap.get(DcMotor.class, "in");
        outtakeLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lo");
        outtakeRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ro");

        // frontServo = hardwareMap.get(CRServo.class, "Front Servo");
        backServo = null;//hardwareMap.get(CRServo.class, "bs");
        gateServo = hardwareMap.get(Servo.class, "gs");
        setGateServo(false);
        // odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odom");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // backServo.setDirection(CRServo.Direction.REVERSE);
        // gateServo.setDirection(Servo.Direction.REVERSE);

        outtakeLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void setAutoCycleList(autoCycles[] autoCycleList) {
        this.autoCycleList = autoCycleList;
    }

    public static double toAngularRate (double RPM, double CPR) {
        return (RPM / 60) * CPR;
    }

    public void teleOpLoop () {
        follower.update();
        if (teleOpState == 0 && gamepad1.yWasPressed() && !follower.isBusy()) {
            teleOpState = 1;
            follower.breakFollowing();
            autoAimed = false;
            autoAim();

        } else if (teleOpState == 0 && gamepad1.xWasPressed() && !follower.isBusy()) {
            teleOpState = 2;
            follower.breakFollowing();
            parkRobot();
        }

        if (teleOpState == 0) {
            // follower.startTeleopDrive(true);
            boolean slowMove = gamepad1.right_trigger > SENSITIVITY;
            double axial;
            double lateral;
            double yaw;

            if (!slowMove) {
                axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                lateral = -gamepad1.left_stick_x;
                yaw = -gamepad1.right_stick_x;
            } else {
                axial = -gamepad1.left_stick_y * 0.5;  // Note: pushing stick forward gives negative value
                lateral = -gamepad1.left_stick_x * 0.5;
                yaw = -gamepad1.right_stick_x * 0.5;
            }

            follower.setTeleOpDrive(axial, lateral, yaw, true);
        } else if (teleOpState == 1) {
            if (autoAimed) {
                autoAimed = false;
                teleOpState = 0;
                follower.breakFollowing();
                follower.startTeleopDrive(true);
            }
        } else if (teleOpState == 2) {
            if (!follower.isBusy()) {
                teleOpState = 0;
                follower.breakFollowing();
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

        boolean gateServoOn = gamepad2.y && outtakeUpToSpeed;
        // setBackServo(backServoOn);
        setGateServo(gateServoOn);

        telemetry.addData("intake status", "%b, %4.2f", intakeOn, INTAKEPOWER);
        telemetry.addData("outtake status", "%b, %4.2f", outtakeOn, angularRate);
        telemetry.addData("gate servo status", gateServoOn);
        telemetry.addData("driving status", teleOpState);
        telemetry.addData("follower is busy", follower.isBusy());

        allTelemetry();
    }

    public void autoAim () {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        Pose currentPose = new Pose(x, y);
        Pose finalPose = new Pose(x + 40, y + 40);
        double heading;

        PathChain autoAimPath;
        if (redAlliance) {
            // NOTE: 138 is better than 144, maybe remove the HL?
            // NOTE: It seems to be aiming too low
            heading = Math.atan((138 - (y)) / (138 - (x))) + Math.PI; // middle of robot, turn to outtak
            // TODO: why is it not going the full way each time?
        }

        else {
            heading = 0 - Math.atan((138 - (y)) / ((x))); // middle of robot, turn to outtake
            // TODO: why is it not going the full way each time?
        }

        autoAimPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, finalPose))
                .setConstantHeadingInterpolation(heading)
                .build();

        Thread thread = new Thread(() -> {
            follower.breakFollowing();
            follower.deactivateAllPIDFs();
            follower.activateHeading();

            follower.followPath(autoAimPath);
            while (true) {
                if (!gamepad1.y) {
                    break;
                }
            }

            follower.breakFollowing();
            follower.activateAllPIDFs();
            autoAimed = true;
        });
        thread.start();
    }

    public void parkRobot () {
        Pose currentPose = follower.getPose();
        Pose parkingPose = (redAlliance) ? P.L_ParkingPose : P.R_ParkingPose;
        PathChain parkPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, parkingPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), parkingPose.getHeading())
                .build();

        follower.breakFollowing();
        follower.followPath(parkPath);
    }

    public PathChain[] buildAutoCyclePaths (Pose start, Pose end) {
        Pose control = new Pose(start.getX(), end.getY(), end.getHeading());
        PathChain path1 = follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setConstantHeadingInterpolation(end.getHeading())
                .setBrakingStart(0.7) // TODO: Make this more accurate, either by increasing or decreasing
                .setBrakingStrength(0.7)
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
                    autoCycle(P.BR_Preload, null, true, true);
                    break;

                case BR_I:
                    paths = buildAutoCyclePaths(P.BR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowI, 0));
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BR_II:
                    paths = buildAutoCyclePaths(P.BR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowII, 0));
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BR_III:
                    paths = buildAutoCyclePaths(P.BR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowIII, 0));
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BR_TRIGGER:
                    paths = buildAutoCyclePaths(P.BR_ScoringPose, P.R_TriggerPose);
                    autoCycle(paths[0], paths[1], false, true);
                    break;

                case BR_END:
                    autoCycle(P.BR_End, null, false, true);
                    break;

                case FR_PRELOAD:
                    autoCycle(P.FR_Preload, null, true, false);
                    break;

                case FR_I:
                    paths = buildAutoCyclePaths(P.FR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowI, 0));
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FR_II:
                    paths = buildAutoCyclePaths(P.FR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowII, 0));
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FR_III:
                    paths = buildAutoCyclePaths(P.FR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowIII, 0));
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FR_TRIGGER:
                    paths = buildAutoCyclePaths(P.FR_ScoringPose, P.R_TriggerPose);
                    autoCycle(paths[0], paths[1], false, false);
                    break;

                case FR_END:
                    autoCycle(P.FR_End, null, false, false);
                    break;

                case BL_PRELOAD:
                    autoCycle(P.BL_Preload, null, true, true);
                    break;

                case BL_I:
                    paths = buildAutoCyclePaths(P.BL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowI, 0));
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BL_II:
                    paths = buildAutoCyclePaths(P.BL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowII, 0));
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BL_III:
                    paths = buildAutoCyclePaths(P.BL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowIII, 0));
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BL_TRIGGER:
                    paths = buildAutoCyclePaths(P.BL_ScoringPose, P.L_TriggerPose);
                    autoCycle(paths[0], paths[1], false, true);
                    break;

                case BL_END:
                    autoCycle(P.BL_End, null, false, true);
                    break;

                case FL_PRELOAD:
                    autoCycle(P.FL_Preload, null, true, false);
                    break;

                case FL_I:
                    paths = buildAutoCyclePaths(P.FL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowI, 0));
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FL_II:
                    paths = buildAutoCyclePaths(P.FL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowII, 0));
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FL_III:
                    paths = buildAutoCyclePaths(P.FL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowIII, 0));
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FL_TRIGGER:
                    paths = buildAutoCyclePaths(P.FL_ScoringPose, P.L_TriggerPose);
                    autoCycle(paths[0], paths[1], false, false);
                    break;

                case FL_END:
                    autoCycle(P.FL_End, null, false, false);
                    break;

                case DELAY_1:
                    if (autoCycleState == -1) {
                        assert true;
                    }

                    else if (autoCycleState == 0) {
                        autoCycleState = -1;
                        Thread thread = new Thread(() -> {
                            try {
                                Thread.sleep(1000);
                            } catch (Exception e) {
                                SLEEPERRORS++;
                            }

                            autoCycleState = 1;
                        });

                        thread.start();
                    }

                    else if (autoCycleState == 1) {
                        autoCycleState = 0;
                        cycleNumber++;
                    }

                default:
                    telemetry.addData("no auto loop state", true);
                    break;
            }
        }

        allTelemetry();

        telemetry.addData("auto number", cycleNumber);
        telemetry.addData("cycle state", autoCycleState);
        telemetry.addData("cycle", currentCycle);
        telemetry.addData("test", triggerCycleNumber == cycleNumber);
    }

    public void autoCycle (PathChain go, PathChain back, boolean useBalls, boolean backShootPower) {
        switch (autoCycleState) {
            case -1:
                break;

            case 0:
                if (go != null) {
                    follower.followPath(go);

                    if (useBalls) {
                        setGateServo(false);
                        setIntake(true);
                    }
                }

                if (triggerCycleNumber == cycleNumber) {
                    autoCycleState = 10;
                }

                else {
                    autoCycleState = 1;
                }

                break;

            case 1:
                if (!follower.isBusy()) {
                    Thread thread = new Thread(() -> {
                        autoCycleState = -1;

                        if (useBalls) {
                            setOuttake(true, backShootPower);
                            try {
                                Thread.sleep(intakeWaitTime) ;
                                setIntake(false);
                            } catch (Exception e) {
                                SLEEPERRORS++;
                            }
                        }

                        if (back != null) {
                            follower.followPath(back);
                        }

                        autoCycleState = 2;
                    });

                    thread.start();
                }

                break;

            case 2:
                if (!useBalls) {
                    autoCycleState = 1000;
                }

                else if (checkUpToSpeed(outtakeLeftMotor, (backShootPower) ? backOuttakeAngularRate : frontOuttakeAngularRate)
                                && !follower.isBusy()) {

                    autoCycleState = -1; // prevent 2 again but also not 0

                    Thread thread = new Thread(() -> {
                        try {
                            Thread.sleep(gateWaitTime);
                        } catch (Exception e) {
                            SLEEPERRORS++;
                        }
                        setGateServo(true);

                        try {
                            Thread.sleep(autoIntakeWaitTime);
                        } catch (Exception e) {
                            SLEEPERRORS++;
                        }
                        setIntakeSlow(true);

                        try {
                            Thread.sleep(shootingTime);
                        } catch (Exception e) {
                            SLEEPERRORS++;
                        }

                        setGateServo(false);
                        setIntakeSlow(false);
                        setOuttake(false, backShootPower);

                        autoCycleState = 1000; // so it isn't in a different thread, it happens on the main line --> only once
                    });

                    thread.start();
                }

                break;

            case 10:
                if (!follower.isBusy()) {
                    Pose currentPose = follower.getPose();
                    Pose triggerPose = (redAlliance) ? P.R_TriggerPose : P.L_TriggerPose;

                    PathChain goTrigger = follower.pathBuilder()
                            .addPath(new BezierLine(currentPose, triggerPose))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), triggerPose.getHeading())
                            .build();

                    follower.followPath(goTrigger);
                    autoCycleState = 11;
                }

                break;

            case 11:
                if (!follower.isBusy()) {
                    Pose currentPose = follower.getPose();
                    Pose triggerPose = (redAlliance) ? P.R_TriggerPose : P.L_TriggerPose;

                    PathChain comeBack = follower.pathBuilder() // go prepare for the back path
                            .addPath(new BezierLine(currentPose, triggerPose))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), triggerPose.getHeading())
                            .build();

                    autoCycleState = -1;

                    Thread thread = new Thread(() -> {
                        try {
                            Thread.sleep(triggerTime);
                        } catch (Exception e) {
                            SLEEPERRORS++;
                        }

                        follower.followPath(comeBack);

                        autoCycleState = 1;
                    });

                    thread.start();
                }

                break;

            case 1000:
                cycleNumber++;
                autoCycleState = 0;

                break;

            default:
                telemetry.addData("auto cycle no state", true);

                break;
        }
    }

    public void init () {
        follower = Constants.createFollower(hardwareMap);

        P.buildMainPoses();
        P.buildOtherAutoPaths(follower);
    }

    public static final startingPoses[] allStartingPoses = startingPoses.values();
    public static int startingSelection = 0;

    public void teleOpInitLoop () {
        if (gamepad1.dpadRightWasPressed()) {
            startingSelection = (startingSelection + 1) % allStartingPoses.length;
        }

        else if (gamepad1.dpadLeftWasPressed()) {
            startingSelection = (startingSelection - 1 + allStartingPoses.length) % allStartingPoses.length;
        }

        if (gamepad1.xWasPressed()) {
            redAlliance = !redAlliance;
        }

        telemetry.addData("starting pose (dPad)", allStartingPoses[startingSelection]);
        telemetry.addData("red alliance (x)", redAlliance);
        telemetry.update();
    }

    public void autoInitLoop () {
        if (gamepad1.dpadLeftWasPressed()) { // init is where the cycle list is called
            triggerCycleNumber = Math.max(-1, triggerCycleNumber - 1);
        }

        else if (gamepad1.dpadRightWasPressed()) {
            triggerCycleNumber = Math.min(autoCycleList.length - 1, triggerCycleNumber + 1);
        }

        autoCycles cycle;
        if (triggerCycleNumber == -1) {
            cycle = autoCycles.NONE;
        }
        else {
            cycle = autoCycleList[triggerCycleNumber];
        }

        if (gamepad1.xWasPressed()) {
            redAlliance = !redAlliance;
        }

        telemetry.addData("the cycle during which the trigger will be triggered (dpads)", cycle);
        telemetry.addData("red alliance (x)", redAlliance);
        telemetry.update();
    }

    public void start () {
        cycleNumber = 0;
        teleOpState = 0;

        follower.setStartingPose(P.startingPose);

        follower.startTeleopDrive(true);
    }

    public void stop () {
        follower.setXVelocity(0);
        follower.setYVelocity(0);

        P.setStoppedPose(follower.getPose());
    }

    public void setIntake (boolean status) {
        intakeMotor.setPower(status ? INTAKEPOWER : 0);
    }

    public void setIntakeSlow (boolean status) {
        intakeMotor.setPower(status ? SLOWINTAKEPOWER : 0);
    }

    public void setOuttake (boolean status, boolean backShoot) {
        double angularRate = (backShoot) ? backOuttakeAngularRate : frontOuttakeAngularRate;
        outtakeLeftMotor.setVelocity(status ? angularRate : 0);
        outtakeRightMotor.setVelocity(status ? angularRate : 0);
    }

    @Deprecated
    public void setBackServo (boolean status) {
        backServo.setPower(status ? SERVOPOWER : 0);
    }

    public void setGateServo (boolean status) {
        gateServo.setPosition(status ? gateOnPosition : gateOffPosition);
    }

    public boolean checkUpToSpeed (DcMotorEx motor, double targetSpeed) {
        return motor.getVelocity() >= UPTOSPEEDTHRESHOLD * targetSpeed;
    }

    public void allTelemetry () {
        telemetry.addData("position x", follower.getPose().getX());
        telemetry.addData("position y", follower.getPose().getY());
        telemetry.addData("position rotation", Math.toDegrees(follower.getHeading()));
        telemetry.addData("errors caught", SLEEPERRORS);
    }
}