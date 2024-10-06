package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;

    public void init(@NonNull HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setDrivetrainPowers(double x, double y, double theta, double modifier) {
        leftFront.setPower((x + y + theta) * modifier);
        leftBack.setPower((x - y + theta) * modifier);
        rightFront.setPower((x - y - theta) * modifier);
        rightBack.setPower((x + y - theta) * modifier);
    }
}
