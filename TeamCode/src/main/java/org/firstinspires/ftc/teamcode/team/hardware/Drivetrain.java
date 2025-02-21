package org.firstinspires.ftc.teamcode.team.hardware;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.URM09Sensor;
import org.firstinspires.ftc.teamcode.powercut.settings;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;

@Config
public class Drivetrain {
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public URM09Sensor leftUpperUS, rightUpperUS;
    public AnalogInput leftLowerUS, rightLowerUS;
    public Rev2mDistanceSensor frontLeftToF, frontRightToF;
    public IMU imu = null;

    public static double basketAlignDistance = 20;
    public static double basketDisengageDistance = 30;
    public static double rungAlignDistance = 25;
    public static double subAlignDistance = 10;
    public static double wallAlignDistance = 15;
    public static PIDCoefficientsEx basketXYCoefficients = new PIDCoefficientsEx(0.05,0,0.005,0,100,0.25);
    public static PIDCoefficientsEx rungYCoefficients = new PIDCoefficientsEx(0.025,0,0.02,0,100,0);
    public static PIDCoefficientsEx basketYawCoefficients = new PIDCoefficientsEx(-1.5,0,0.005,0,0.5,0);
    public static PIDCoefficientsEx yawLockCoefficients = new PIDCoefficientsEx(-1.25,0,0.5,0,0.5,0);
    public static PIDCoefficientsEx subYCoefficients = new PIDCoefficientsEx(-0.025,0,0.02,0,100,0);

    private PIDEx XAlignPID = new PIDEx(basketXYCoefficients);
    private PIDEx YAlignPID = new PIDEx(basketXYCoefficients);
    private PIDEx RungAlignPID = new PIDEx(rungYCoefficients);
    private PIDEx SubAlignPID = new PIDEx(subYCoefficients);
    private PIDEx basketYawAlignPID = new PIDEx(basketYawCoefficients);
    AngleController basketYawController = new AngleController(basketYawAlignPID);
    private PIDEx yawAlignPID = new PIDEx(yawLockCoefficients);
    AngleController yawController = new AngleController(yawAlignPID);

    private ElapsedTime radialTime = new ElapsedTime();
    private double lastYaw, radialVelocity;

    private static double yawLockThetaDeadzone = 0.05;
    private static double yawLockRadialDeadzone = 5;
    private double yawLock;
    private boolean yawLockActive;

    private double lastX = 0;
    private double lastY = 0;
    private double lastTheta = 0;

    public static double yawAlignDeadzone = 2.5;
    public static double xyAlignDeadzone = 5;
    public static double rungAlignDeadzone = 2.5;
    public static double ToFCentreDistance = 180;
    public static double USCentreDistance = 32.8;

    public static double USYawFilterGain = 0.5;
    LowPassFilter USYawFilter = new LowPassFilter(USYawFilterGain);

    public static double ToFYawFilterGain = 0.5;
    LowPassFilter ToFYawFilter = new LowPassFilter(ToFYawFilterGain);
    public boolean isDriveAction = false;

    public void init(@NonNull HardwareMap hardwareMap) {
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

        radialTime.reset();
        lastYaw = getYaw();
    }

    public double getYaw() {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        updateRadialVelocity();
        return robotOrientation.getYaw(AngleUnit.DEGREES);
    }

    public double getRadialVelocity() {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);

        radialVelocity = Math.abs(lastYaw - yaw) / radialTime.seconds();
        lastYaw = yaw;
        radialTime.reset();

        return radialVelocity;
    }

    public void updateRadialVelocity() {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);

        radialVelocity = Math.abs(lastYaw - yaw) / radialTime.seconds();
        lastYaw = yaw;
        radialTime.reset();
    }

    private void rawXYThetaMod(double x, double y, double theta, double modifier) {

        double leftFrontPower = (y+x+theta) * modifier;
        double leftBackPower = (y-x+theta) * modifier;
        double rightFrontPower = (y-x-theta) * modifier;
        double rightBackPower = (y+x-theta) * modifier;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        updateRadialVelocity();
    }

    public void setDrivetrainPowers(double x, double y, double theta, double modifier) {
        if (Math.abs(x) > 0.05 || Math.abs(y) > 0.05 || Math.abs(theta) > 0.05) {
            isDriveAction = false;
        }
        if (!isDriveAction) {
            double yawRad = Math.toRadians(getYaw());

            if ((Math.abs(radialVelocity) < yawLockRadialDeadzone) && (Math.abs(theta) < yawLockThetaDeadzone)) {
                if (!yawLockActive) {
                    yawLock = getYaw();
                    yawLockActive = true;
                }

                theta = yawController.calculate(yawLock, yawRad);

                if (Math.abs(theta) < 0.05) {
                    theta = 0;
                }
            } else {
                yawLockActive = false;
            }

            if ((Math.abs(x - lastX) > settings.driveCacheAmount) || (Math.abs(y - lastY) > settings.driveCacheAmount) || (Math.abs(theta - lastTheta) > settings.driveCacheAmount)) {
                rawXYThetaMod(x, y, theta, modifier);
                lastX = x;
                lastY = y;
                lastTheta = theta;
            }
        }
    }

    public void setDrivetrainPowers(double x, double y, double theta, double modifier, boolean rotate) {
        if (Math.abs(x) > 0.05 || Math.abs(y) > 0.05 || Math.abs(theta) > 0.05) {
            isDriveAction = false;
        }

        if (!isDriveAction) {
            double yawRad = Math.toRadians(getYaw());

            if ((Math.abs(radialVelocity) < yawLockRadialDeadzone) && (Math.abs(theta) < yawLockThetaDeadzone)) {
                if (!yawLockActive) {
                    yawLock = getYaw();
                    yawLockActive = true;
                }

                theta = yawController.calculate(yawLock, yawRad);

                if (Math.abs(theta) < 0.05) {
                    theta = 0;
                }
            } else {
                yawLockActive = false;
            }

            yawRad = rotate ? yawRad : 0;

            double x_rotated = x * Math.cos(-yawRad) - y * Math.sin(-yawRad);
            double y_rotated = x * Math.sin(-yawRad) + y * Math.cos(-yawRad);

            if ((Math.abs(x_rotated - lastX) > settings.driveCacheAmount) || (Math.abs(y_rotated - lastY) > settings.driveCacheAmount) || (Math.abs(theta - lastTheta) > settings.driveCacheAmount)) {
                rawXYThetaMod(x_rotated, y_rotated, theta, modifier);
                lastX = x_rotated;
                lastY = y_rotated;
                lastTheta = theta;
            }

        }
    }

    public void kill() {
        setDrivetrainPowers(0,0,0,1);
    }

    public double getYawFromToF() {
        double leftDistance = frontLeftToF.getDistance(DistanceUnit.MM);
        double rightDistance = frontRightToF.getDistance(DistanceUnit.MM);

        double difference = rightDistance - leftDistance;

        return ToFYawFilter.estimate(Math.toDegrees(Math.asin(difference/ToFCentreDistance)));
    }

    public double getYawFromUS() {
        double leftLowerMVout = leftLowerUS.getVoltage() * 1000;
        double rightLowerMVout = rightLowerUS.getVoltage() * 1000;

        double leftDistance = (leftLowerMVout * 520) / 3300;
        double rightDistance = (rightLowerMVout * 520) / 3300;

        double difference = rightDistance - leftDistance;

        return USYawFilter.estimate(Math.toDegrees(Math.asin(difference / USCentreDistance)));
    }

    //Alignment Actions

    public class AlignBasket implements Action {
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

            double x = XAlignPID.calculate(basketAlignDistance, leftDistance);
            double y = YAlignPID.calculate(basketAlignDistance, rightDistance);
            double theta = yawController.calculate(Math.toRadians(-45), yawRad);
            double x_rotated = x * Math.cos(-yawRad) - y * Math.sin(-yawRad);
            double y_rotated = x * Math.sin(-yawRad) + y * Math.cos(-yawRad);


            Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(new Vector2d((-70.5 + (x / 2.54) + 8.34), (-70.5 + (y / 2.54)) + 8.34), yawRad));

            if (!isDriveAction || ((Math.abs(yawError) < yawAlignDeadzone) && (Math.abs(basketAlignDistance - leftDistanceRaw) < xyAlignDeadzone) && (Math.abs(basketAlignDistance - rightDistanceRaw) < xyAlignDeadzone))) {
                rawXYThetaMod(0,0,0,1);
                isDriveAction = false;
                return false;
            } else {
                rawXYThetaMod(x_rotated, y_rotated, theta, 1);
                return true;
            }
        }
    }

    public Action alignBasket() {
        return new AlignBasket();
    }

    public class DisengageBasket implements Action {
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

            double x = XAlignPID.calculate(basketDisengageDistance, leftDistance);
            double y = YAlignPID.calculate(basketDisengageDistance, rightDistance);
            double theta = yawController.calculate(Math.toRadians(-45), yawRad);
            double x_rotated = x * Math.cos(-yawRad) - y * Math.sin(-yawRad);
            double y_rotated = x * Math.sin(-yawRad) + y * Math.cos(-yawRad);


            Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(new Vector2d((-70.5 + (x / 2.54) + 8.34), (-70.5 + (y / 2.54)) + 8.34), yawRad));

            if (!isDriveAction || ((Math.abs(yawError) < yawAlignDeadzone) && (Math.abs(basketDisengageDistance - leftDistanceRaw) < xyAlignDeadzone) && (Math.abs(basketDisengageDistance - rightDistanceRaw) < xyAlignDeadzone))) {
                rawXYThetaMod(0,0,0,1);
                isDriveAction = false;
                return false;
            } else {
                rawXYThetaMod(x_rotated, y_rotated, theta, 1);
                return true;
            }
        }
    }

    public Action disengageBasket() {
        return new DisengageBasket();
    }

    public class AlignRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double yaw = getYaw();
            double yawRad = Math.toRadians(yaw);
            double yawError = (180 - yaw);

            double leftLowerMVout = leftLowerUS.getVoltage() * 1000;
            double rightLowerMVout = rightLowerUS.getVoltage() * 1000;

            double leftDistance = (leftLowerMVout*520)/3300;
            double rightDistance = (rightLowerMVout*520)/3300;

            double y = RungAlignPID.calculate(rungAlignDistance, (rightDistance+leftDistance)/2);
            double theta = yawController.calculate(Math.toRadians(180), yawRad);


            if (!isDriveAction || ((Math.abs(yawError) < yawAlignDeadzone) && (Math.abs(rungAlignDistance - leftDistance) < rungAlignDeadzone) && (Math.abs(rungAlignDistance - rightDistance) < rungAlignDeadzone))) {
                rawXYThetaMod(0,0,0,1);
                isDriveAction = false;
                return false;
            } else {
                rawXYThetaMod(0, y, theta, 1);;
                return true;
            }
        }
    }

    public Action alignRung() {
        return new AlignRung();
    }

    public class AlignWall implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double yaw = getYaw();
            double yawRad = Math.toRadians(yaw);
            double yawError = (0 - yaw);

            double leftLowerMVout = leftLowerUS.getVoltage() * 1000;
            double rightLowerMVout = rightLowerUS.getVoltage() * 1000;

            double leftDistance = (leftLowerMVout*520)/3300;
            double rightDistance = (rightLowerMVout*520)/3300;

            double y = RungAlignPID.calculate(wallAlignDistance, (rightDistance+leftDistance)/2);
            double theta = yawController.calculate(Math.toRadians(0), yawRad);

            if (!isDriveAction || ((Math.abs(yawError) < yawAlignDeadzone) && (Math.abs(wallAlignDistance - leftDistance) < rungAlignDeadzone) && (Math.abs(settings.wallAlignDistance - rightDistance) < rungAlignDeadzone))) {
                rawXYThetaMod(0,0,0,1);
                isDriveAction = false;
                return false;
            } else {
                rawXYThetaMod(0, y, theta, 1);;
                return true;
            }
        }
    }

    public Action alignWall() {
        return new AlignWall();
    }

    public class AlignSubmersible implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double yaw = getYaw();
            double yawRad = Math.toRadians(yaw);
            double yawError = (0 - yaw);

            double leftDistance = frontLeftToF.getDistance(DistanceUnit.MM)/10;
            double rightDistance = frontRightToF.getDistance(DistanceUnit.MM)/10;

            double y = SubAlignPID.calculate(subAlignDistance, (rightDistance+leftDistance)/2);
            double theta = 0;
            if (yaw < -45) {
                theta = yawController.calculate(Math.toRadians(-90), yawRad);
            } else if (yaw > -45 && yaw < 45) {
                theta = yawController.calculate(Math.toRadians(0), yawRad);
            } else {
                theta = yawController.calculate(Math.toRadians(90), yawRad);
            }

            packet.addLine("Theta " + theta);


            if (!isDriveAction || (Math.abs(yawError) < yawAlignDeadzone) && (Math.abs(subAlignDistance - leftDistance) < rungAlignDeadzone) && (Math.abs(subAlignDistance - rightDistance) < rungAlignDeadzone)) {
                rawXYThetaMod(0,0,0,1);
                isDriveAction = false;
                return false;
            } else {
                rawXYThetaMod(0, y, theta, 1);;
                return true;
            }
        }
    }

    public Action alignSubmersible() {
        return new AlignSubmersible();
    }
}
