package org.firstinspires.ftc.teamcode.powercut.hardware;


import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Lift {
    public DcMotorEx leftLift, rightLift;

    public DigitalChannel liftStop;

    public static PIDCoefficientsEx liftCoefficients = new PIDCoefficientsEx(0.0035,0.00,0.0000, 500, 150, 0);
    private PIDEx liftPID = new PIDEx(liftCoefficients);

    //public static PIDCoefficientsEx liftHangCoefficients = new PIDCoefficientsEx(2,1,0.00000, 1000, 0, 0);
    public static double liftEqCoef = 0.001;
    public static int allowableExtensionDeficit= 50;
    public static int allowableHangDeficit= 10;

    public static double liftHoldPower = 0.05;
    public static double liftHangHoldPower = -0.2;
    public static double liftHangPullPower = -1;
    public static double liftRetractPower = -0.25;

    public static int liftTopBasket = 2800;
//    public static int liftBottomBasket = 1250;
    public static int liftTopRung = 1200;
    public static int liftTopRungAttached = 800;
//    public static int liftBottomRung = 1250;
//    public static int liftBottomRungAttached = 1050;
    public static int liftClearance = 500;

    public static int liftPreHang = 650;
    public static int liftHang = 0;

    public static int liftRetraction = 0;


    private double lastLiftPower = 0;
    public boolean isLiftAvailable = true;
    public boolean isDescending = false;






    // resets and inits
    protected void init(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        liftStop = hardwareMap.get(DigitalChannel.class, "liftStop");

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift.setDirection(DcMotor.Direction.REVERSE);
    }

    public double getLeftLiftCurrent() {
        return leftLift.getCurrent(CurrentUnit.AMPS);
    }

    public double getRightLiftCurrent() {
        return rightLift.getCurrent(CurrentUnit.AMPS);
    }

    public void setLiftPower(double power) {
            isDescending = power < 0;

            if (power > 1.0) {
                power = 1.0;
            }

            if (isDescending && !liftStop.getState()) {
                power = 0;
            }

            if (Math.abs(power - lastLiftPower) < 0.04) {

            } else {
                if (!liftStop.getState()) {
                    leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                int leftLiftPos = leftLift.getCurrentPosition();
                int rightLiftPos = rightLift.getCurrentPosition();
                double error = leftLiftPos - rightLiftPos;

                double equaliser = error * liftEqCoef;

                double leftPowerRaw = power;
                double rightPowerRaw = power + equaliser;

                double leftPower = 0.0;
                double rightPower = 0.0;

                if (rightPowerRaw > 1.0) {
                    double multiplier = 1.0 / rightPowerRaw;
                    leftPower = leftPowerRaw * multiplier;
                    rightPower = rightPowerRaw * multiplier;
                } else {
                    leftPower = leftPowerRaw;
                    rightPower = rightPowerRaw;
                }

                leftLift.setPower(leftPower);
                rightLift.setPower(rightPower);
            }

    }

    public void kill() {
        leftLift.setPower(0);
        rightLift.setPower(0);

        if (!liftStop.getState()) {
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        isLiftAvailable = true;
    }

    public class liftTopBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;


            if (Math.abs(averagePos - liftTopBasket) < allowableExtensionDeficit) {
                kill();
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(liftTopBasket, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftTopBasket() {
        return new liftTopBasket();
    }

//    public class liftBottomBasket implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            isLiftAvailable = false;
//            int leftLiftPos = leftLift.getCurrentPosition();
//            int rightLiftPos = rightLift.getCurrentPosition();
//            int averagePos = (leftLiftPos + rightLiftPos)/2;
//
//            if (Math.abs(leftLiftPos - liftBottomBasket) < allowableExtensionDeficit || Math.abs(rightLiftPos - liftBottomBasket) < allowableExtensionDeficit) {
//                setLiftPower(liftHoldPower);
//                isLiftAvailable = true;
//                return false;
//            } else {
//
//                double power = liftPID.calculate(liftBottomBasket, averagePos);
//
//                setLiftPower(power);
//
//                return true;
//            }
//        }
//    }
//
//    public Action liftBottomBasket() {
//        return new liftBottomBasket();
//    }

    public class liftTopRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;

            if (Math.abs(averagePos - liftTopRung) < allowableExtensionDeficit) {
                kill();
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(liftTopRung, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftTopRung() {
        return new liftTopRung();
    }

    public class liftTopRungAttached implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;

            if (Math.abs(averagePos - liftTopRungAttached) < allowableExtensionDeficit) {
                kill();
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(liftTopRungAttached, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftTopRungAttached() {
        return new liftTopRungAttached();
    }

    public class liftClearance implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;


            if (Math.abs(averagePos - liftClearance) < allowableExtensionDeficit) {
                kill();
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(liftClearance, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftClearance() {
        return new liftClearance();
    }

//    public class liftBottomRung implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            isLiftAvailable = false;
//            int leftLiftPos = leftLift.getCurrentPosition();
//            int rightLiftPos = rightLift.getCurrentPosition();
//            int averagePos = (leftLiftPos + rightLiftPos)/2;
//
//            if (Math.abs(leftLiftPos - liftBottomRung) < allowableExtensionDeficit || Math.abs(rightLiftPos - liftBottomRung) < allowableExtensionDeficit) {
//                setLiftPower(liftHoldPower);
//                isLiftAvailable = true;
//                return false;
//            } else {
//
//                double power = liftPID.calculate(liftBottomRung, averagePos);
//
//                setLiftPower(power);
//
//                return true;
//            }
//        }
//    }
//
//    public Action liftBottomRung() {
//        return new liftBottomRung();
//    }
//
//    public class liftBottomRungAttached implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            isLiftAvailable = false;
//            int leftLiftPos = leftLift.getCurrentPosition();
//            int rightLiftPos = rightLift.getCurrentPosition();
//            int averagePos = (leftLiftPos + rightLiftPos)/2;
//
//            if (Math.abs(leftLiftPos - liftBottomRungAttached) < allowableExtensionDeficit || Math.abs(rightLiftPos - liftBottomRungAttached) < allowableExtensionDeficit) {
//                setLiftPower(liftHoldPower);
//                isLiftAvailable = true;
//                return false;
//            } else {
//
//                double power = liftPID.calculate(liftBottomRungAttached, averagePos);
//
//                setLiftPower(power);
//
//                return true;
//            }
//        }
//    }
//
//    public Action liftBottomRungAttached() {
//        return new liftBottomRungAttached();
//    }

    public class liftRetract implements Action {

        boolean isCooked = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;

            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;
            double power = 0;

            if ((Math.abs(averagePos - liftRetraction) < allowableExtensionDeficit) || !liftStop.getState()) {
                kill();
                return false;
            } else {
                if ((Math.abs(averagePos - liftRetraction) < allowableExtensionDeficit) || isCooked) {
                    isCooked = true;
                    power = liftRetractPower;
                } else {
                    power = liftPID.calculate(liftRetraction, averagePos);
                }
                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftRetract() {
        return new liftRetract();
    }

    public class LiftRetractSensor implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;

            if (!liftStop.getState()) {
                kill();
                return false;
            } else {
                setLiftPower(liftRetractPower);

                return true;
            }
        }
    }

    public Action liftRetractSensor() {
        return new LiftRetractSensor();
    }

    public class liftPreHang implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;

            if (Math.abs(averagePos - liftPreHang) < allowableExtensionDeficit) {
                kill();
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(liftPreHang, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftPreHang() {
        return new liftPreHang();
    }

    public class liftHang implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;


            if (Math.abs(averagePos - liftHang) < allowableHangDeficit) {
                setLiftPower(liftHangHoldPower);
                return false;
            } else {

                double power = liftHangPullPower;

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftHang() {
        return new liftHang();
    }
}
