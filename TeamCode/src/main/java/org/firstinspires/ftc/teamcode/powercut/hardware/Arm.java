package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.powercut.settings;

public class Arm {
    private Servo leftArm = null;
    private Servo rightArm = null;
    private Servo grip = null;

    public void init(HardwareMap hardwareMap) {
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        grip = hardwareMap.get(Servo.class, "grip");
    }

    public class raiseArm implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftArm.setPosition(settings.armRaised);
            rightArm.setPosition(settings.armRaised);

            return false;
        }
    }

    public Action raiseArm() {
        return new raiseArm();
    }

    public class lowerArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftArm.setPosition(settings.armLowered);
            rightArm.setPosition(settings.armRaised);

            return false;
        }
    }

    public Action lowerArm() {
        return new lowerArm();
    }

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
