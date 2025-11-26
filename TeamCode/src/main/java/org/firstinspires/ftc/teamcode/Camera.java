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

public class Camera {

    private HardwareMap hardwareMap;
    private int i;
    public Camera (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
}
