package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.URM09Sensor;
import org.firstinspires.ftc.teamcode.powercut.settings;

import java.util.List;

@Config
public class Drivetrain {
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public URM09Sensor leftUltrasonic, rightUltrasonic;
    public Rev2mDistanceSensor frontToF;
    public IMU imu = null;

    public static double yawAlignDeadzone = 2;
    public static double xyAlignDeadzone= 5;

    public void init(@NonNull HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftUltrasonic = hardwareMap.get(URM09Sensor.class, "leftUltrasonic");
        rightUltrasonic = hardwareMap.get(URM09Sensor.class, "rightUltrasonic");
        frontToF = hardwareMap.get(Rev2mDistanceSensor.class, "frontDistance");

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        leftUltrasonic.setMeasurementMode(true);
        rightUltrasonic.setMeasurementMode(true);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setDrivetrainPowers(double x, double y, double theta, double modifier) {
        double leftFrontPower = (y+x+theta) * modifier;
        double leftBackPower = (y-x+theta) * modifier;
        double rightFrontPower = (y-x-theta) * modifier;
        double rightBackPower = (y+x-theta) * modifier;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    public double getYaw() {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        return robotOrientation.getYaw(AngleUnit.DEGREES);
    }

    public class alignBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double yaw = getYaw();
            double yawRad = Math.toRadians(yaw);
            double yawError = (-45 - yaw);
            double leftDistance = leftUltrasonic.getDistance();
            double rightDistance = rightUltrasonic.getDistance();
            double xError = settings.basketAlignDistance - leftDistance;
            double yError = settings.basketAlignDistance - rightDistance;

            if (Math.abs(yawError) > yawAlignDeadzone) {
                setDrivetrainPowers(0, 0, yawError * settings.basketAlignYawKp, 1);
            } else {
                double x = xError * settings.basketXYKp;
                double y = yError * settings.basketXYKp;
                double x_rotated = x * Math.cos(-yawRad) - y * Math.sin(-yawRad);
                double y_rotated = x * Math.sin(-yawRad) + y * Math.cos(-yawRad);
                setDrivetrainPowers(x_rotated, y_rotated, 0, 1);
            }

            if ((Math.abs(yawError) < yawAlignDeadzone) && (Math.abs(xError) < xyAlignDeadzone) && (Math.abs(yError) < xyAlignDeadzone)) {
                setDrivetrainPowers(0,0,0,1);
                return false;
            } else {
                return true;
            }
        }
    }

    public Action alignBasket() {
        return new alignBasket();
    }
}
