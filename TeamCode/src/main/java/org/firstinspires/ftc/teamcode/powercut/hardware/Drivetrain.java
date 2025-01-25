package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.URM09Sensor;
import org.firstinspires.ftc.teamcode.powercut.settings;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;

import java.util.List;

@Config
public class Drivetrain {
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public URM09Sensor leftUpperUS, rightUpperUS;
    public AnalogInput leftLowerUS, rightLowerUS;
    public Rev2mDistanceSensor frontLeftToF, frontRightToF;


    public IMU imu = null;
    private PIDEx XYAlignPID = new PIDEx(settings.basketXYCoefficients);
    private PIDEx yawAlignPID = new PIDEx(settings.basketYawCoefficients);

    public static double yawAlignDeadzone = 5;
    public static double xyAlignDeadzone = 5;
    public static double ToFCentreDistance = 180;
    public static double USCentreDistance = 33;

    public void init(@NonNull HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftUpperUS = hardwareMap.get(URM09Sensor.class, "leftUpperUS");
        rightUpperUS = hardwareMap.get(URM09Sensor.class, "rightUpperUS");
        leftLowerUS = hardwareMap.get(AnalogInput.class, "leftLowerUS");
        rightLowerUS = hardwareMap.get(AnalogInput.class, "rightLowerUS");
        frontLeftToF = hardwareMap.get(Rev2mDistanceSensor.class, "frontLeftToF");
        frontRightToF = hardwareMap.get(Rev2mDistanceSensor.class, "frontRightToF");




        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        leftUpperUS.setMeasurementMode(true);
        rightUpperUS.setMeasurementMode(true);

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

    public double getYawFromToF() {
        double leftDistance = frontLeftToF.getDistance(DistanceUnit.MM);
        double rightDistance = frontRightToF.getDistance(DistanceUnit.MM);

        double difference = rightDistance - leftDistance;

        return Math.toDegrees(Math.asin(difference/ToFCentreDistance));
    }

    public double getYawFromUS() {
        double leftLowerMVout = leftLowerUS.getVoltage() * 1000;
        double rightLowerMVout = rightLowerUS.getVoltage() * 1000;

        double leftDistance = (leftLowerMVout*520)/3300;
        double rightDistance = (rightLowerMVout*520)/3300;

        double difference = rightDistance - leftDistance;

        return Math.toDegrees(Math.asin(difference/USCentreDistance));
    }

    public class alignBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double yaw = getYaw();
            double yawRad = Math.toRadians(yaw);
            double yawError = (-45 - yaw);
            double yawErrorRad = Math.toRadians(yawError);

            double leftDistanceRaw = leftUpperUS.getDistance();
            double rightDistanceRaw = rightUpperUS.getDistance();

            double leftDistance = Math.cos(yawErrorRad) * leftDistanceRaw;
            double rightDistance = Math.cos(yawErrorRad) * rightDistanceRaw;

            double x = XYAlignPID.calculate(settings.basketAlignDistance, leftDistance);
            double y = XYAlignPID.calculate(settings.basketAlignDistance, rightDistance);
            double theta = yawAlignPID.calculate(-45, yaw);
            double x_rotated = x * Math.cos(-yawRad) - y * Math.sin(-yawRad);
            double y_rotated = x * Math.sin(-yawRad) + y * Math.cos(-yawRad);
            setDrivetrainPowers(x_rotated, y_rotated, theta, 1);

            Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(new Vector2d((-70.5 + (x / 2.54) + 8.34), (-70.5 + (y / 2.54)) + 8.34), yawRad));

            if ((Math.abs(yawError) < yawAlignDeadzone) && (Math.abs(settings.basketAlignDistance - leftDistanceRaw) < xyAlignDeadzone) && (Math.abs(settings.basketAlignDistance - rightDistanceRaw) < xyAlignDeadzone)) {
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

    public class alignRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double yaw = getYaw();
            double yawError = (0 - yaw);

            packet.addLine("Yaw Error " + yawError);

            double leftLowerMVout = leftLowerUS.getVoltage() * 1000;
            double rightLowerMVout = rightLowerUS.getVoltage() * 1000;

            double leftDistance = (leftLowerMVout*520)/3300;
            double rightDistance = (rightLowerMVout*520)/3300;

            double y = XYAlignPID.calculate(settings.rungAlignDistance, (rightDistance+leftDistance)/2);
            double theta = yawAlignPID.calculate(0, yaw);
            packet.addLine("Theta " + theta);
            setDrivetrainPowers(0, y, 0, 1);;

            if ((Math.abs(yawError) < yawAlignDeadzone) && (Math.abs(settings.rungAlignDistance - leftDistance) < xyAlignDeadzone) && (Math.abs(settings.rungAlignDistance - rightDistance) < xyAlignDeadzone)) {
                setDrivetrainPowers(0,0,0,1);
                return false;
            } else {
                return true;
            }
        }
    }

    public Action alignRung() {
        return new alignRung();
    }
}
