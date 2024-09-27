package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.powercut.settings;

public class Lift {
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;

    private BasicPID liftPID = new BasicPID(settings.liftCoefficients);

    // resets and inits
    public void init(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public class liftExtend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int leftLiftPos = leftLift.getCurrentPosition();
            int rightLiftPos = rightLift.getCurrentPosition();


            if (Math.abs(leftLiftPos - settings.liftExtension) < settings.allowableExtensionDeficit && Math.abs(rightLiftPos - settings.liftExtension) < settings.allowableExtensionDeficit) {
                return false;
            } else {

                double power = liftPID.calculate(settings.liftExtension, leftLiftPos);

                leftLift.setPower(power);
                rightLift.setPower(power);

                leftLiftPos = leftLift.getCurrentPosition();
                rightLiftPos = rightLift.getCurrentPosition();

                return Math.abs(leftLiftPos - settings.liftExtension) >= settings.allowableExtensionDeficit || Math.abs(rightLiftPos - settings.liftExtension) >= settings.allowableExtensionDeficit;
            }
        }
    }

    public Action liftExtend() {
        return new liftExtend();
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
