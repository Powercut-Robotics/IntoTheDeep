package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.powercut.settings;

public class Outtake {
    private ServoImplEx leftArm, rightArm, grip;


    public void init(HardwareMap hardwareMap) {
        leftArm = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightArm = hardwareMap.get(ServoImplEx.class, "rightArm");
        grip = hardwareMap.get(ServoImplEx.class, "grip");


        leftArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightArm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        leftArm.setDirection(ServoImplEx.Direction.REVERSE);
    }

    //ARM
    public class raiseArm implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftArm.setPosition(settings.upperArmRaised);
            rightArm.setPosition(settings.upperArmRaised);

            return false;
        }
    }

    public Action raiseArm() {
        return new raiseArm();
    }

    public class specIntakeArm implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftArm.setPosition(settings.upperArmIntake);
            rightArm.setPosition(settings.upperArmIntake);

            return false;
        }
    }

    public Action specIntakeArm() {
        return new specIntakeArm();
    }

    public class lowerArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftArm.setPosition(settings.upperArmLowered);
            rightArm.setPosition(settings.upperArmLowered);

            return false;
        }
    }

    public Action lowerArm() {
        return new lowerArm();
    }

    //GRIP

    public class closeGrip implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            grip.setPosition(settings.gripClosed);
            return false;
        }
    }

    public Action closeGrip() {
        return new closeGrip();
    }

    public class openGrip implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            grip.setPosition(settings.gripOpen);

            return false;
        }
    }
    public Action openGrip() {
        return new openGrip();
    }
}
