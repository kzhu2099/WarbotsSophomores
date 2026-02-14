package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final DcMotor intakeMotor;
    private final DcMotorEx outtakeLeftMotor;
    private final DcMotorEx outtakeRightMotor;
    private final CRServo backServo;
    private final CRServo gateServo;

    public static final double WIDTH = 18;
    public static final double LENGTH = 18;
    public static final double HW = WIDTH / 2;
    public static final double HL = LENGTH / 2;

    //------IMPORTANT SHOOTING VARIABLES------\\
    private static final double SHOOTINTAKEPOWER = 0.85;
    private static final double FRONTOUTTAKERPM = 3400;
    private static final double BACKOUTTAKERPM = 3540;
    private static final int shootingTimeOnA = 350; // intake on
    private static final int shootingTimeOffA = 50; // intake off
    private static final int shootingTimeOnB = 600; // also the best
    private static final int shootingTimeOffB = 0; // also pretty good though could be shorter
    private static final double gateOffPosition = 0.225;
    private static final double gateOnPosition = 0.015;
    private static final int gateTimeOn = 3000;
    
    //------MOTORS / SERVOS------\\
    // private static final double EMERGENCYOUTTAKERPM = 5500;
    private static final double SENSITIVITY = 0.1;
    private static final double SERVOPOWER = 1;
    private static final double INTAKEPOWER = 0.8;
    private static final double OUTTAKECPR = 28;
    private static final double frontOuttakeAngularRate = toAngularRate(FRONTOUTTAKERPM, OUTTAKECPR);
    private static final double backOuttakeAngularRate = toAngularRate(BACKOUTTAKERPM, OUTTAKECPR);
    private static final double UPTOSPEEDTHRESHOLD = 0.99; // measuring velocity is not always accurate so this will be triggered

    //------AUTO CONTROL------\\
    private autoCycles[] autoCycleList;
    private static final int baseDeltaY = 0; // should be zero unless bad
    private static final int autoCycleAimTime = 500;
    private static final int autoCycleAimAmount = 2;
    private static final double autoCycleSpeed = 0.8;
    private static final int triggerTime = 500;
    private static int[] triggerCycleNumbers = new int[] {-1};
    
    //------GAME PLAYING-------\\
    private static double SLEEPERRORS = 0;
    private Follower follower;
    private boolean monitorThreads = true;
    private static boolean redAlliance;
    private int autoCycleNumber = 0;
    private int teleOpDriveState = 0;
    private int autoCycleState = 0;
    private int teleOpShootState = 0;
    private boolean robotCentric = false;
    private boolean autoAimed = false;
    
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, boolean _redAlliance) {
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
        gateServo = hardwareMap.get(CRServo.class, "gs");
        setGateServoOld(false);
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
        telemetry.update()
        ;
    }

    public void startMonitorThreads () {
        Thread thread = new Thread(() -> {
            while (monitorThreads) {
                assert true;
            }
        });

        thread.start();
    }

    public void setAutoCycleList(autoCycles[] autoCycleList) {
        this.autoCycleList = autoCycleList;
    }

    public static double toAngularRate (double RPM, double CPR) {
        return (RPM / 60) * CPR;
    }
    
    public PathChain getAutoAimPath () {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        Pose currentPose = new Pose(x, y);
        Pose finalPose = new Pose(x + 40, y + 40);
        double heading;

        PathChain autoAimPath;
        if (redAlliance) {
            // NOTE: It seems to be aiming too low
            heading = Math.atan((144 - (y)) / (144 - (x))) + Math.PI; // middle of robot, turn to outtak
        }

        else {
            heading = 0 - Math.atan((144 - (y)) / ((x))); // middle of robot, turn to outtake
        }

        autoAimPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, finalPose))
                .setConstantHeadingInterpolation(heading)
                .build();

        return autoAimPath;
    }

    public void setToTurnMode () {
        follower.breakFollowing();
        follower.deactivateAllPIDFs();
        follower.activateHeading();
    }

    public void setToRegularMode () {
        follower.breakFollowing();
        follower.activateAllPIDFs();
    }

    public void autoAim () {
        PathChain autoAimPath = getAutoAimPath();

        Thread thread = new Thread(() -> {
            follower.breakFollowing();
            setToTurnMode();
            follower.followPath(autoAimPath);
            while (true) {
                if (!gamepad1.y) {
                    break;
                }
            }
            follower.breakFollowing();
            setToRegularMode();
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

    public void teleOpLoop () {
        follower.update();

        // Driving State
        if (teleOpDriveState == 0 && gamepad1.yWasPressed() && !follower.isBusy()) {
            teleOpDriveState = 1;
            follower.breakFollowing();
            autoAimed = false;
            autoAim();
        }

        else if (teleOpDriveState == 0 && gamepad1.xWasPressed() && !follower.isBusy()) {
            teleOpDriveState = 2;
            follower.breakFollowing();
            parkRobot();
        }

        if (gamepad1.bWasPressed()) {
            robotCentric = !robotCentric;
        }

        if (teleOpDriveState == 0) {
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

            if (!robotCentric) {
                axial *= (redAlliance) ? 1 : -1;
                lateral *= (redAlliance) ? 1 : -1;
            }

            follower.setTeleOpDrive(axial, lateral, yaw, robotCentric);
        }

        else if (teleOpDriveState == 1) {
            if (autoAimed) {
                autoAimed = false;
                teleOpDriveState = 0;
                follower.breakFollowing();
                follower.startTeleopDrive(true);
            }
        }

        else if (teleOpDriveState == 2) {
            if (!follower.isBusy()) {
                teleOpDriveState = 0;
                follower.breakFollowing();
                follower.startTeleopDrive(true);
            }
        }

        // Intake / Outtake
        boolean intakeOn = false;
        boolean gateServoOn = false;

        boolean outtakeOn = gamepad2.right_trigger > SENSITIVITY;
        boolean backShoot = gamepad2.right_bumper;

        setOuttake(outtakeOn, backShoot);

        boolean outtakeUpToSpeed = checkUpToSpeed(outtakeLeftMotor, backShoot) && checkUpToSpeed(outtakeRightMotor, backShoot);

        if (teleOpShootState == 0 && gamepad2.y && outtakeUpToSpeed) {
            teleOpShootState = 1;
        }

        if (teleOpShootState == 0) {
            intakeOn = gamepad2.left_trigger > SENSITIVITY;
            intakeMotor.setPower(intakeOn ? INTAKEPOWER : 0);

            // gateServoOn = gamepad2.y && outtakeUpToSpeed;
            // setGateServoOld(gateServoOn);
        }

        else if (teleOpShootState == 1) {
            if (gamepad2.yWasReleased()) {
                teleOpShootState = 0;
            }
            else if (outtakeUpToSpeed) {
                teleOpShootState = 2;
                Thread thread = new Thread(() -> {
                    setIntakeSlow(true);
                    while (teleOpShootState == 2) {
                        try {
                            setGateServo(true);
                            /*setGateServoOld(true); Thread.sleep(shootingTimeOnB);
                            setGateServoOld(false); Thread.sleep(shootingTimeOffB);
                            setIntakeSlow(true); Thread.sleep(shootingTimeOnA);
                            setIntakeSlow(false); Thread.sleep(shootingTimeOffA);*/
                        } catch (Exception e) {
                            SLEEPERRORS++;
                        }
                    }
                    setIntakeSlow(false);
                    setGateServo(false);
                });

                thread.start();
            }
        }

        else if (teleOpShootState == 2) {
            if (gamepad2.yWasReleased()) {
                teleOpShootState = 0;
            }
        }

        telemetry.addData("intake status", "%b, %4.2f", intakeOn, INTAKEPOWER);
        telemetry.addData("outtake status / backshoot", "%b, %b", outtakeOn, backShoot);
        telemetry.addData("gate servo status", gateServoOn);
        telemetry.addData("driving status", teleOpDriveState);
        telemetry.addData("shooting status", teleOpShootState);
        telemetry.addData("follower is busy", follower.isBusy());
        telemetry.addData("motor speed L/R", "%4.2f, %4.2f", outtakeLeftMotor.getVelocity(), outtakeRightMotor.getVelocity());
        telemetry.addData("robot centric", "%b", robotCentric);

        updateAllTelemetry();
    }

    public PathChain[] buildAutoCyclePaths (Pose start, Pose end, boolean backAuto) {
        double deltaY = baseDeltaY * ((backAuto) ? -1 : 1);
        Pose control = new Pose(start.getX(), end.getY() + deltaY, end.getHeading());
        end = new Pose(end.getX(), end.getY() + deltaY, end.getHeading());
        PathChain path1 = follower.pathBuilder()
                .addPath(new BezierCurve(start, control, control, control, end))
                .setConstantHeadingInterpolation(end.getHeading())
                //.setBrakingStart(2)
                //.setBrakingStrength(0.1)
                //.setGlobalDeceleration(0.6)
                //.setTranslationalConstraint(1)
                //.setVelocityConstraint(5)
                .build();

        PathChain path2 = follower.pathBuilder()
                .addPath(new BezierCurve(end, control, start))
                .setConstantHeadingInterpolation(start.getHeading())
                //.setBrakingStart(0.5)
                //.setBrakingStrength(0.5)
                //.setGlobalDeceleration(0.4)
                //.setTranslationalConstraint(2)
                /*.addParametricCallback(0.99, () -> {
                    follower.breakFollowing();
                })*/
                .setHeadingConstraint(Math.toRadians(10))
                .build();

        return new PathChain[] {path1, path2};
    }

    public void autoShoot (int endState, boolean backShoot, boolean nextCycle) {
        setOuttake(true, backShoot);

        if (checkUpToSpeed(outtakeLeftMotor, backShoot) && checkUpToSpeed(outtakeRightMotor, backShoot) && !follower.isBusy()) {
            autoCycleState = -2;

            Thread thread = new Thread(() -> {
                for (int i = 0; i < autoCycleAimAmount; i++) {
                    PathChain autoAimPath = getAutoAimPath();
                    follower.breakFollowing();
                    setToTurnMode();
                    follower.followPath(autoAimPath);

                    try {
                        Thread.sleep(autoCycleAimTime);
                    } catch (Exception e) {
                        SLEEPERRORS++;
                    }

                    follower.breakFollowing();
                    setToRegularMode();
                }

                autoCycleState = -3;

                try {
                    for (int i = 0; i < 3; i++) {
                        try {
                            setGateServoOld(true); Thread.sleep(shootingTimeOnB);
                            setGateServoOld(false); Thread.sleep(shootingTimeOffB);
                            if (i < 2) {
                                setIntakeSlow(true); Thread.sleep(shootingTimeOnA);
                                setIntakeSlow(false); Thread.sleep(shootingTimeOffA);
                            }
                        } catch (Exception e) {
                            SLEEPERRORS++;
                        }
                    }
                } catch (Exception e) {
                    SLEEPERRORS++;
                }

                setGateServoOld(false);
                setIntakeSlow(false);
                setOuttake(false, backShoot);

                if (nextCycle) {
                    autoCycleNumber++;
                }

                autoCycleState = endState; // so it isn't in a different thread, it happens on the main line --> only once
            });

            thread.start();
        }
    }

    public void autoLoop () {
        follower.update();

        autoCycles currentCycle;

        if (autoCycleNumber < autoCycleList.length) {
            currentCycle = autoCycleList[autoCycleNumber];
        }

        else {
            currentCycle = autoCycles.NONE;
        }

        if ((!follower.isBusy() || autoCycleState == 0) && autoCycleState >= 0) { // won't change if it is busy, the == 0 shouldn't happen
            PathChain[] paths;
            switch (currentCycle) {
                case NONE:
                    break;

                case BR_INIT:
                    autoCycle(P.BR_Preload, null, false, true);
                    setOuttake(true, true);
                    break;

                case BR_PRELOAD:
                    autoShoot(0, true, true);
                    break;

                case BR_I:
                    paths = buildAutoCyclePaths(P.BR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowI, Math.toRadians(0)), true);
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BR_II:
                    paths = buildAutoCyclePaths(P.BR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowII, Math.toRadians(0)), true);
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BR_III:
                    paths = buildAutoCyclePaths(P.BR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowIII, Math.toRadians(0)), true);
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BR_III_PICKUP:
                    paths = buildAutoCyclePaths(P.BR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowIII, Math.toRadians(0)), true);
                    autoCycle(paths[0], null, true, true);
                    break;

                case BR_TRIGGER:
                    paths = buildAutoCyclePaths(P.BR_ScoringPose, P.R_TriggerPose, true);
                    autoCycle(paths[0], paths[1], false, true);
                    break;

                case BR_TRIGGER_PICKUP:
                    break;

                case BR_END:
                    autoCycle(P.BR_End, null, false, true);
                    break;

                case BL_INIT:
                    autoCycle(P.BL_Preload, null, false, true);
                    setOuttake(true, true);
                    break;

                case BL_PRELOAD:
                    autoShoot(0, true, true);
                    break;

                case BL_I:
                    paths = buildAutoCyclePaths(P.BL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowI, Math.toRadians(180)), true);
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BL_II:
                    paths = buildAutoCyclePaths(P.BL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowII, Math.toRadians(180)), true);
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BL_III:
                    paths = buildAutoCyclePaths(P.BL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowIII, Math.toRadians(180)), true);
                    autoCycle(paths[0], paths[1], true, true);
                    break;

                case BL_III_PICKUP:
                    paths = buildAutoCyclePaths(P.BL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowIII, Math.toRadians(180)), true);
                    autoCycle(paths[0], null, true, true);
                    break;

                case BL_TRIGGER:
                    paths = buildAutoCyclePaths(P.BL_ScoringPose, P.L_TriggerPose, true);
                    autoCycle(paths[0], paths[1], false, true);
                    break;

                case BL_TRIGGER_PICKUP:
                    break;

                case BL_END:
                    autoCycle(P.BL_End, null, false, true);
                    break;

                case FR_INIT:
                    autoCycle(P.FR_Preload, null, false, false);
                    setOuttake(true, false);
                    break;

                case FR_PRELOAD:
                    autoShoot(0, false, true);
                    break;

                case FR_I:
                    paths = buildAutoCyclePaths(P.FR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowI, Math.toRadians(0)), false);
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FR_II:
                    paths = buildAutoCyclePaths(P.FR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowII, Math.toRadians(0)), false);
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FR_III:
                    paths = buildAutoCyclePaths(P.FR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowIII, Math.toRadians(0)), false);
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FR_I_PICKUP:
                    paths = buildAutoCyclePaths(P.FR_ScoringPose, new Pose(72 + P.cycleEndDX, P.rowI, Math.toRadians(0)), false);
                    autoCycle(paths[0], null, true, false);
                    break;

                case FR_TRIGGER:
                    paths = buildAutoCyclePaths(P.FR_ScoringPose, P.R_TriggerPose, true);
                    autoCycle(paths[0], paths[1], false, false);
                    break;

                case FR_TRIGGER_PICKUP:
                    break;

                case FR_END:
                    autoCycle(P.BR_End, null, false, false);
                    break;

                case FL_INIT:
                    autoCycle(P.FL_Preload, null, false, false);
                    setOuttake(true, false);
                    break;

                case FL_PRELOAD:
                    autoShoot(0, false, true);
                    break;

                case FL_I:
                    paths = buildAutoCyclePaths(P.FL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowI, Math.toRadians(180)), false);
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FL_II:
                    paths = buildAutoCyclePaths(P.FL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowII, Math.toRadians(180)), false);
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FL_III:
                    paths = buildAutoCyclePaths(P.FL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowIII, Math.toRadians(180)), false);
                    autoCycle(paths[0], paths[1], true, false);
                    break;

                case FL_I_PICKUP:
                    paths = buildAutoCyclePaths(P.FL_ScoringPose, new Pose(72 - P.cycleEndDX, P.rowI, Math.toRadians(180)), false);
                    autoCycle(paths[0], null, true, false);
                    break;

                case FL_TRIGGER:
                    paths = buildAutoCyclePaths(P.FL_ScoringPose, P.L_TriggerPose, true);
                    autoCycle(paths[0], paths[1], false, false);
                    break;

                case FL_TRIGGER_PICKUP:
                    break;

                case FL_END:
                    autoCycle(P.BL_End, null, false, false);
                    break;

                case DELAY_1:
                    if (autoCycleState == 0) {
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
                        autoCycleNumber++;
                    }

                default:
                    telemetry.addData("no auto loop state", true);
                    break;
            }
        }

        updateAllTelemetry();

        telemetry.addData("auto number", autoCycleNumber);
        telemetry.addData("cycle state", autoCycleState);
        telemetry.addData("cycle", currentCycle);
        telemetry.addData("test", checkTrigger(autoCycleNumber));
    }

    public boolean checkTrigger(int cycleNumber) {
        for (int i: triggerCycleNumbers) {
            if (i == cycleNumber) {
                return true;
            }
        }

        return false;
    }

    public void autoCycle (PathChain go, PathChain back, boolean useBalls, boolean backShoot) {
        switch (autoCycleState) {
            case 0:
                if (go != null) {
                    follower.followPath(go, autoCycleSpeed, true);

                    if (useBalls) {
                        setGateServoOld(false);
                        setIntake(true);
                    }

                    if (checkTrigger(autoCycleNumber)) {
                        autoCycleState = 10;
                    }

                    else {
                        autoCycleState = 1;
                    }
                }

                else {
                    autoCycleState = 1000;
                }

                break;

            case 1:
                if (!follower.isBusy()) {
                    if (useBalls) {
                        setOuttake(true, backShoot);
                        setIntake(false);
                    }

                    if (back != null) {
                        follower.followPath(back, autoCycleSpeed, true);
                        autoCycleState = 2;
                    }

                    else {
                        setOuttake(false, false);
                        autoCycleState = 1000;
                    }
                }

                break;

            case 2:
                if (!useBalls) {
                    autoCycleState = 1000;
                }

                else {
                    autoShoot(1000, true, false);
                }

                break;

            case 10:
                if (!follower.isBusy()) {
                    Pose currentPose = follower.getPose();
                    Pose triggerPose = (redAlliance) ? P.R_TriggerPose : P.L_TriggerPose;
                    double xdx = ((redAlliance) ? -1 : 1) * 3;
                    Pose controlPose = new Pose(currentPose.getX() + xdx, (triggerPose.getY() + currentPose.getY()) / 2, triggerPose.getHeading());
                    PathChain goTrigger = follower.pathBuilder()
                            .addPath(new BezierCurve(currentPose, controlPose, triggerPose))
                            .setConstantHeadingInterpolation(triggerPose.getHeading())
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
                autoCycleNumber++;
                autoCycleState = 0;

                break;

            default:
                telemetry.addData("auto cycle no state", true);

                break;
        }
    }

    public void init () {
        follower = Constants.createFollower(hardwareMap);
        setGateServoOld(false);
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

    public void setTriggerCycles(int[] triggers) {
        triggerCycleNumbers = triggers;
    }

    public void autoInitLoop () {

        if (gamepad1.xWasPressed()) {
            redAlliance = !redAlliance;
        }

        telemetry.addData("red alliance (x)", redAlliance);
        telemetry.update();
    }

    public void start () {
        autoCycleNumber = 0;
        teleOpDriveState = 0;

        follower.setStartingPose(P.startingPose);

        P.buildMainPoses();
        P.buildOtherAutoPaths(follower);

        follower.startTeleopDrive(true);

        setGateServoOld(false);

        monitorThreads = true;
    }

    public void stop () {
        follower.breakFollowing();

        P.setStoppedPose(follower.getPose());

        monitorThreads = false;
    }

    public void setIntake (boolean status) {
        intakeMotor.setPower(status ? INTAKEPOWER : 0);
    }

    public void setIntakeSlow (boolean status) {
        intakeMotor.setPower(status ? SHOOTINTAKEPOWER : 0);
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

    @Deprecated
    public void setGateServoOld (boolean status) {
        // gateServo.setPosition(status ? gateOnPosition : gateOffPosition);
    }

    public void setGateServo (boolean status) {
        gateServo.setPower((status) ? 1 : -1);
    }

    public boolean checkUpToSpeed (DcMotorEx motor, boolean backShoot) {
        double angularRate = (backShoot) ? backOuttakeAngularRate : frontOuttakeAngularRate;
        return motor.getVelocity() >= UPTOSPEEDTHRESHOLD * angularRate;
    }

    public void updateAllTelemetry () {
        telemetry.addData("position x", follower.getPose().getX());
        telemetry.addData("position y", follower.getPose().getY());
        telemetry.addData("position rotation", Math.toDegrees(follower.getHeading()));
        telemetry.addData("errors caught", SLEEPERRORS);
    }
}