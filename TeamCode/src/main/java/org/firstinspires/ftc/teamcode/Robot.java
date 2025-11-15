/* Copyright (c) 2021 FIRST. All rights reserved.
 * TESTING TESTING
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private DcMotor intakeMotor;
    private DcMotorEx outtakeLeftMotor;
    private DcMotorEx outtakeRightMotor;

    // private CRServo frontServo;
    private CRServo backServo;
    
    // private static final double OUTTAKEPOWER = 0.95;
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

    public Robot (HardwareMap _hardwareMap, Telemetry _telemetry) {
        this.hardwareMap = _hardwareMap;
        this.telemetry = _telemetry;

        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        intakeMotor = hardwareMap.get(DcMotor.class, "in");
        outtakeLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lo");
        outtakeRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ro");

        // frontServo = hardwareMap.get(CRServo.class, "Front Servo");
        backServo = hardwareMap.get(CRServo.class, "bs");

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

    public void teleOpDrive (Gamepad gamepad1, Gamepad gamepad2) {
        double max;

        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        boolean intakeOn = gamepad2.left_trigger > SENSITIVITY;
        boolean outtakeOn = gamepad2.right_trigger > SENSITIVITY;
        boolean backShoot = gamepad2.right_bumper;
        boolean emergencyShoot = gamepad2.left_bumper;
        boolean backServoOn = gamepad2.y;
        boolean slowMove = gamepad1.right_trigger > SENSITIVITY;

        double slowSpeed = slowMove ? 0.5 : 1.0;
        double frontLeftPower  = slowSpeed * (axial + lateral + yaw);
        double frontRightPower = slowSpeed * (axial - lateral - yaw);
        double backLeftPower   = slowSpeed * (axial - lateral + yaw);
        double backRightPower  = slowSpeed * (axial + lateral - yaw);

        // double frontServoPower = (frontServoOn) ? SERVOPOWER : 0;
        double backServoPower = (backServoOn) ? SERVOPOWER : 0;

        max = Math.max(1, Math.abs(frontLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        intakeMotor.setPower(intakeOn ? INTAKEPOWER : 0);

        double angularRate = (backShoot) ? backOuttakeAngularRate : frontOuttakeAngularRate;
        angularRate = (emergencyShoot) ? emergencyOuttakeAngularRate : angularRate;

        outtakeLeftMotor.setVelocity(outtakeOn ? angularRate : 0);
        outtakeRightMotor.setVelocity(outtakeOn ? angularRate : 0);

        // frontServo.setPower(frontServoPower);
        backServo.setPower(backServoPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.addData("intake status", "%b, %4.2f", intakeOn, INTAKEPOWER);
        telemetry.addData("outtake status", "%b, %4.2f", outtakeOn, angularRate);
        telemetry.addData("outtake back shoot status", "%b", backShoot);
        telemetry.addData("outtake emergency shoot status", "%b", emergencyShoot);

        // telemetry.addData("front servo status", "%b, %4.2f", frontServoOn, frontServoPower);
        telemetry.addData("back servo status", "%b, %4.2f", backServoOn, backServoPower);
        telemetry.update();
    }

    public void setPowers (double[] powers) {
        frontLeftDrive.setPower(powers[0]);
        frontRightDrive.setPower(powers[1]);
        backLeftDrive.setPower(powers[2]);
        backRightDrive.setPower(powers[3]);
    }

    public void delay (double time) {
        double startTime = runtime.seconds();
        while (runtime.seconds() - startTime < time) {
            telemetry.addData("delay", runtime.seconds() - startTime);
        }
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
}
