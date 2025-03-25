package org.firstinspires.ftc.teamcode.powercut.hardware;

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
    public static PIDCoefficientsEx yawLockCoefficients = new PIDCoefficientsEx(-0.8,0,0,0,0.5,0);
    public static PIDCoefficientsEx subYCoefficients = new PIDCoefficientsEx(-0.025,0,0.02,0,100,0);

    private PIDEx XAlignPID = new PIDEx(basketXYCoefficients);
    private PIDEx YAlignPID = new PIDEx(basketXYCoefficients);
    private PIDEx RungAlignPID = new PIDEx(rungYCoefficients);
    private PIDEx SubAlignPID = new PIDEx(subYCoefficients);

    private PIDEx yawAlignPID = new PIDEx(yawLockCoefficients);
    AngleController yawController = new AngleController(yawAlignPID);

    private ElapsedTime radialTime = new ElapsedTime();
    private double lastYaw, radialVelocity;

    private static double yawLockThetaDeadzone = 0.02;
    private static double yawLockRadialDeadzone = 0.15;
    private double yawLock = 0;
    private boolean yawLockActive = false;

    private double lastX = 0;
    private double lastY = 0;
    private double lastTheta = 0;

    public static double driveCacheAmount = 0.01;

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

    protected void init(@NonNull HardwareMap hardwareMap) {
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

        imu.resetYaw();



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

        return robotOrientation.getYaw(AngleUnit.RADIANS);
    }

    public double getRadialVelocity() {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.RADIANS);

        radialVelocity = Math.abs(lastYaw - yaw) / radialTime.seconds();
        lastYaw = yaw;
        radialTime.reset();

        return radialVelocity;
    }

    public void updateRadialVelocity() {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.RADIANS);

        radialVelocity = Math.abs(lastYaw - yaw) / radialTime.seconds();
        lastYaw = yaw;
        radialTime.reset();
    }

    public double getLowerLeftUS() {
        double MVout = leftLowerUS.getVoltage() * 1000;

        return (MVout * 520) / 3300;
    }

    public double getLowerRightUS() {
        double MVout = rightLowerUS.getVoltage() * 1000;

        return (MVout * 520) / 3300;
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

    public void setDrivetrainPowers(double x, double y, double theta, double modifier, double yaw) {
        if (Math.abs(x) > 0.05 || Math.abs(y) > 0.05 || Math.abs(theta) > 0.05) {
            isDriveAction = false;
        }

        double normalYaw = yaw;
        if (yaw > Math.PI) {
            normalYaw = yaw - 2 * Math.PI;
        }

        if (!isDriveAction) {
            //double yaw = getYaw();

            if ((Math.abs(theta) < yawLockThetaDeadzone)) {
                if (!yawLockActive) {
                    yawLock = normalYaw;
                }

                if (!yawLockActive && getRadialVelocity() < yawLockRadialDeadzone) {
                    yawLockActive = true;
                }

                theta = yawController.calculate(yawLock, normalYaw);

                if (Math.abs(theta) < 0.05) {
                    theta = 0;
                }
            } else {
                yawLockActive = false;
            }

            if ((Math.abs(x - lastX) > driveCacheAmount) || (Math.abs(y - lastY) > driveCacheAmount) || (Math.abs(theta - lastTheta) > driveCacheAmount)) {
                rawXYThetaMod(x, y, theta, modifier);
                lastX = x;
                lastY = y;
                lastTheta = theta;
            }
        }
    }

    public void setDrivetrainPowers(double x, double y, double theta, double modifier, double yaw, boolean rotate) {
        double normalYaw = yaw;
        if (yaw > Math.PI) {
            normalYaw = yaw - 2 * Math.PI;
        }

        if (Math.abs(x) > 0.05 || Math.abs(y) > 0.05 || Math.abs(theta) > 0.05) {
            isDriveAction = false;
        }

        if (!isDriveAction) {
            //double yaw = getYaw();

            if ((Math.abs(theta) < yawLockThetaDeadzone)) {
                if (!yawLockActive) {
                    yawLock = normalYaw;
                }

                if (!yawLockActive && getRadialVelocity() < yawLockRadialDeadzone) {
                    yawLockActive = true;
                }

                theta = yawController.calculate(yawLock, normalYaw);

                if (Math.abs(theta) < 0.05) {
                    theta = 0;
                }
            } else {
                yawLockActive = false;
            }

            normalYaw = rotate ? normalYaw : 0;

            double x_rotated = x * Math.cos(-normalYaw) - y * Math.sin(-normalYaw);
            double y_rotated = x * Math.sin(-normalYaw) + y * Math.cos(-normalYaw);

            if ((Math.abs(x_rotated - lastX) > driveCacheAmount) || (Math.abs(y_rotated - lastY) > driveCacheAmount) || (Math.abs(theta - lastTheta) > driveCacheAmount)) {
                rawXYThetaMod(x_rotated, y_rotated, theta, modifier);
                lastX = x_rotated;
                lastY = y_rotated;
                lastTheta = theta;
            }

        } else {
            yawLockActive = false;
        }
    }

    public void kill() {
        setDrivetrainPowers(0,0,0,0,1);
    }

    public double getYawFromToF() {
        double leftDistance = frontLeftToF.getDistance(DistanceUnit.MM);
        double rightDistance = frontRightToF.getDistance(DistanceUnit.MM);

        double difference = rightDistance - leftDistance;

        return ToFYawFilter.estimate(Math.toDegrees(Math.asin(difference/ToFCentreDistance)));
    }



    //Alignment Actions

    public class AlignBasket implements Action {
        private boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                isDriveAction = true;
                first = false;
            }
            double yaw = getYaw();
            double yawError = ((-0.25 * Math.PI) - yaw);

            double leftDistanceRaw = leftUpperUS.getDistance();
            double rightDistanceRaw = rightUpperUS.getDistance();

            double leftDistance = Math.cos(yawError) * leftDistanceRaw;
            double rightDistance = Math.cos(yawError) * rightDistanceRaw;

            double x = XAlignPID.calculate(basketAlignDistance, leftDistance);
            double y = YAlignPID.calculate(basketAlignDistance, rightDistance);
            double theta = yawController.calculate((-0.25 * Math.PI), yaw);
            double x_rotated = x * Math.cos(-yaw) - y * Math.sin(-yaw);
            double y_rotated = x * Math.sin(-yaw) + y * Math.cos(-yaw);


            Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(new Vector2d((-70.5 + (x / 2.54) + 8.34), (-70.5 + (y / 2.54)) + 8.34), yaw));

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
        private boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                isDriveAction = true;
                first = false;
            }

            double yaw = getYaw();
            double yawError = ((-0.25 * Math.PI) - yaw);

            double leftDistanceRaw = leftUpperUS.getDistance();
            double rightDistanceRaw = rightUpperUS.getDistance();

            double leftDistance = Math.cos(yawError) * leftDistanceRaw;
            double rightDistance = Math.cos(yawError) * rightDistanceRaw;

            double x = XAlignPID.calculate(basketDisengageDistance, leftDistance);
            double y = YAlignPID.calculate(basketDisengageDistance, rightDistance);
            double theta = yawController.calculate(Math.toRadians((-0.25 * Math.PI)), yaw);
            double x_rotated = x * Math.cos(-yaw) - y * Math.sin(-yaw);
            double y_rotated = x * Math.sin(-yaw) + y * Math.cos(-yaw);


            Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(new Vector2d((-70.5 + (x / 2.54) + 8.34), (-70.5 + (y / 2.54)) + 8.34), yaw));

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
        private boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                isDriveAction = true;
                first = false;
            }

            double yaw = getYaw();
            double yawError = (Math.PI - yaw);

            double leftDistance = getLowerLeftUS();
            //double rightDistance = getLowerRightUS();

            double y = RungAlignPID.calculate(rungAlignDistance, leftDistance);
            double theta = yawController.calculate(Math.toRadians(180), yaw);


            if (!isDriveAction || ((Math.abs(yawError) < yawAlignDeadzone) && (Math.abs(rungAlignDistance - leftDistance) < rungAlignDeadzone))) {
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
        private boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                isDriveAction = true;
                first = false;
            }

            double yaw = getYaw();
            double yawError = (0 - yaw);

            double leftDistance = getLowerLeftUS();


            double y = RungAlignPID.calculate(wallAlignDistance, leftDistance);
            double theta = yawController.calculate(Math.toRadians(0), yaw);

            if (!isDriveAction || ((Math.abs(yawError) < yawAlignDeadzone) && (Math.abs(wallAlignDistance - leftDistance) < rungAlignDeadzone))) {
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
        private boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                isDriveAction = true;
                first = false;
            }

            double yaw = getYaw();
            double yawError = (0 - yaw);

            double leftDistance = frontLeftToF.getDistance(DistanceUnit.MM)/10;
            double rightDistance = frontRightToF.getDistance(DistanceUnit.MM)/10;

            double y = SubAlignPID.calculate(subAlignDistance, (rightDistance+leftDistance)/2);
            double theta = 0;
            if (yaw < (-0.25 * Math.PI)) {
                theta = yawController.calculate(Math.toRadians((-0.5 * Math.PI)), yaw);
            } else if (yaw > (-0.25 * Math.PI) && yaw < (0.25 * Math.PI)) {
                theta = yawController.calculate(Math.toRadians(0), yaw);
            } else {
                theta = yawController.calculate(Math.toRadians(0.5 * Math.PI), yaw);
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
