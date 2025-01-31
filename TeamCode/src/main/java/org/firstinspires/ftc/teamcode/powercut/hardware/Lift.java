package org.firstinspires.ftc.teamcode.powercut.hardware;

import static org.firstinspires.ftc.teamcode.powercut.settings.liftCoefficients;
import static org.firstinspires.ftc.teamcode.powercut.settings.liftHangCoefficients;
import static org.firstinspires.ftc.teamcode.powercut.settings.liftHangPower;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.powercut.settings;

public class Lift {
    public DcMotorEx leftLift, rightLift;

    public DigitalChannel liftStop;

    private PIDEx liftPID = new PIDEx(liftCoefficients);
    private PIDEx liftHangPID = new PIDEx(liftHangCoefficients);

    public boolean isLiftAvailable = true;


    private double lastLiftPower = 0;
    public boolean isDescending = false;

    // resets and inits
    public void init(HardwareMap hardwareMap) {
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

                double equaliser = error * settings.liftEqCoef;

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


            if (Math.abs(leftLiftPos - settings.liftTopBasket) < settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftTopBasket) < settings.allowableExtensionDeficit) {
                setLiftPower(settings.liftHoldPower);
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(settings.liftTopBasket, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftTopBasket() {
        return new liftTopBasket();
    }

    public class liftBottomBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;

            if (Math.abs(leftLiftPos - settings.liftBottomBasket) < settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftBottomBasket) < settings.allowableExtensionDeficit) {
                setLiftPower(settings.liftHoldPower);
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(settings.liftBottomBasket, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftBottomBasket() {
        return new liftBottomBasket();
    }

    public class liftTopRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;

            if (Math.abs(leftLiftPos - settings.liftTopRung) < settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftTopRung) < settings.allowableExtensionDeficit) {
                setLiftPower(settings.liftHoldPower);
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(settings.liftTopRung, averagePos);

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

            if (Math.abs(leftLiftPos - settings.liftTopRungAttached) < settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftTopRungAttached) < settings.allowableExtensionDeficit) {
                setLiftPower(settings.liftHoldPower);
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(settings.liftTopRungAttached, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftTopRungAttached() {
        return new liftTopRungAttached();
    }

    public class liftBottomRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;

            if (Math.abs(leftLiftPos - settings.liftBottomRung) < settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftBottomRung) < settings.allowableExtensionDeficit) {
                setLiftPower(settings.liftHoldPower);
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(settings.liftBottomRung, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftBottomRung() {
        return new liftBottomRung();
    }

    public class liftBottomRungAttached implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;

            if (Math.abs(leftLiftPos - settings.liftBottomRungAttached) < settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftBottomRungAttached) < settings.allowableExtensionDeficit) {
                setLiftPower(settings.liftHoldPower);
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(settings.liftBottomRungAttached, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftBottomRungAttached() {
        return new liftBottomRungAttached();
    }

    public class liftRetract implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;

            if (Math.abs(leftLiftPos - settings.liftRetraction) < settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftRetraction) < settings.allowableExtensionDeficit) {
                kill();
                return false;
            } else {

                double power = liftPID.calculate(settings.liftRetraction, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftRetract() {
        return new liftRetract();
    }

    public class liftPreHang implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isLiftAvailable = false;
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();
            int averagePos = (leftLiftPos + rightLiftPos)/2;

            if (Math.abs(leftLiftPos - settings.liftPreHang) < settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftPreHang) < settings.allowableExtensionDeficit) {
                setLiftPower(settings.liftHoldPower);
                isLiftAvailable = true;
                return false;
            } else {

                double power = liftPID.calculate(settings.liftPreHang, averagePos);

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


            if (Math.abs(leftLiftPos - settings.liftRetraction) < settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftRetraction) < settings.allowableExtensionDeficit) {
                setLiftPower(liftHangPower);
                return false;
            } else {

                double power = liftHangPID.calculate(settings.liftRetraction, averagePos);

                setLiftPower(power);

                return true;
            }
        }
    }

    public Action liftHang() {
        return new liftHang();
    }
}
