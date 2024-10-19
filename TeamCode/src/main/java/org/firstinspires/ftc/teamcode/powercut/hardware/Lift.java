package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.powercut.settings;

public class Lift {
    public DcMotorEx leftLift = null;
    public DcMotorEx rightLift = null;

    private BasicPID liftPID = new BasicPID(settings.liftCoefficients);

    // resets and inits
    public void init(HardwareMap hardwareMap) {
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

    public class liftTopBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();


            if (Math.abs(leftLiftPos - settings.liftTopBasket) < settings.allowableExtensionDeficit && Math.abs(rightLiftPos - settings.liftTopBasket) < settings.allowableExtensionDeficit) {
                return false;
            } else {

                double power = liftPID.calculate(settings.liftTopBasket, leftLiftPos);

                leftLift.setPower(power);
                rightLift.setPower(power);

                leftLiftPos = leftLift.getCurrentPosition();
                rightLiftPos = rightLift.getCurrentPosition();

                return Math.abs(leftLiftPos - settings.liftTopBasket) >= settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftTopBasket) >= settings.allowableExtensionDeficit;
            }
        }
    }

    public Action liftTopBasket() {
        return new liftTopBasket();
    }

    public class liftBottomBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();


            if (Math.abs(leftLiftPos - settings.liftBottomBasket) < settings.allowableExtensionDeficit && Math.abs(rightLiftPos - settings.liftBottomBasket) < settings.allowableExtensionDeficit) {
                return false;
            } else {

                double power = liftPID.calculate(settings.liftBottomBasket, leftLiftPos);

                leftLift.setPower(power);
                rightLift.setPower(power);

                leftLiftPos = leftLift.getCurrentPosition();
                rightLiftPos = rightLift.getCurrentPosition();

                return Math.abs(leftLiftPos - settings.liftBottomBasket) >= settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftBottomBasket) >= settings.allowableExtensionDeficit;
            }
        }
    }

    public Action liftBottomBasket() {
        return new liftBottomBasket();
    }

    public class liftTopRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();


            if (Math.abs(leftLiftPos - settings.liftTopRung) < settings.allowableExtensionDeficit && Math.abs(rightLiftPos - settings.liftTopRung) < settings.allowableExtensionDeficit) {
                return false;
            } else {

                double power = liftPID.calculate(settings.liftTopRung, leftLiftPos);

                leftLift.setPower(power);
                rightLift.setPower(power);

                leftLiftPos = leftLift.getCurrentPosition();
                rightLiftPos = rightLift.getCurrentPosition();

                return Math.abs(leftLiftPos - settings.liftTopRung) >= settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftTopRung) >= settings.allowableExtensionDeficit;
            }
        }
    }

    public Action liftTopRung() {
        return new liftTopRung();
    }

    public class liftTopRungAttached implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();


            if (Math.abs(leftLiftPos - settings.liftTopRungAttached) < settings.allowableExtensionDeficit && Math.abs(rightLiftPos - settings.liftTopRungAttached) < settings.allowableExtensionDeficit) {
                return false;
            } else {

                double power = liftPID.calculate(settings.liftTopRungAttached, leftLiftPos);

                leftLift.setPower(power);
                rightLift.setPower(power);

                leftLiftPos = leftLift.getCurrentPosition();
                rightLiftPos = rightLift.getCurrentPosition();

                return Math.abs(leftLiftPos - settings.liftTopRungAttached) >= settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftTopRungAttached) >= settings.allowableExtensionDeficit;
            }
        }
    }

    public Action liftTopRungAttached() {
        return new liftTopRungAttached();
    }

    public class liftBottomRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();


            if (Math.abs(leftLiftPos - settings.liftBottomRung) < settings.allowableExtensionDeficit && Math.abs(rightLiftPos - settings.liftBottomRung) < settings.allowableExtensionDeficit) {
                return false;
            } else {

                double power = liftPID.calculate(settings.liftBottomRung, leftLiftPos);

                leftLift.setPower(power);
                rightLift.setPower(power);

                leftLiftPos = leftLift.getCurrentPosition();
                rightLiftPos = rightLift.getCurrentPosition();

                return Math.abs(leftLiftPos - settings.liftBottomRung) >= settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftBottomRung) >= settings.allowableExtensionDeficit;
            }
        }
    }

    public Action liftBottomRung() {
        return new liftBottomRung();
    }

    public class liftBottomRungAttached implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();


            if (Math.abs(leftLiftPos - settings.liftBottomRungAttached) < settings.allowableExtensionDeficit && Math.abs(rightLiftPos - settings.liftBottomRungAttached) < settings.allowableExtensionDeficit) {
                return false;
            } else {

                double power = liftPID.calculate(settings.liftBottomRungAttached, leftLiftPos);

                leftLift.setPower(power);
                rightLift.setPower(power);

                leftLiftPos = leftLift.getCurrentPosition();
                rightLiftPos = rightLift.getCurrentPosition();

                return Math.abs(leftLiftPos - settings.liftBottomRungAttached) >= settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftBottomRungAttached) >= settings.allowableExtensionDeficit;
            }
        }
    }

    public Action liftBottomRungAttached() {
        return new liftBottomRungAttached();
    }

    public class liftRetract implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();

            if (Math.abs(leftLiftPos - settings.liftRetraction) < settings.allowableExtensionDeficit && Math.abs(rightLiftPos - settings.liftRetraction) < settings.allowableExtensionDeficit) {
                return false;
            } else {

                double power = liftPID.calculate(settings.liftRetraction, leftLiftPos);

                leftLift.setPower(power);
                rightLift.setPower(power);

                leftLiftPos = leftLift.getCurrentPosition();
                rightLiftPos = rightLift.getCurrentPosition();

                return Math.abs(leftLiftPos - settings.liftRetraction) >= settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftRetraction) >= settings.allowableExtensionDeficit;
            }
        }
    }

    public Action liftRetract() {
        return new liftRetract();
    }
}
