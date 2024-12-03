package org.firstinspires.ftc.teamcode.powercut.hardware;

import static org.firstinspires.ftc.teamcode.powercut.settings.liftCoefficients;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.powercut.settings;

import java.util.List;

public class Lift {
    public DcMotorEx leftLift, rightLift;

    private PIDEx liftPID = new PIDEx(liftCoefficients);

    public boolean isLiftAvailable = true;

    // resets and inits
    public void init(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

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
            if (power > 1.0) {
                power = 1.0;
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

    public void kill() {
        leftLift.setPower(0);
        rightLift.setPower(0);
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
                kill();
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
                kill();
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
                kill();
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
                kill();
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
                kill();
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
                kill();
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
}
