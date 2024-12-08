package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.powercut.settings;

import java.util.List;

public class Arm {
    private ServoImplEx leftArm, rightArm, grip;

    public enum sampleColour {
        BLUE,
        RED,
        YELLOW,
        NONE
    }

    public ColorRangeSensor colourRangeSensor = null;

    public void init(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftArm = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightArm = hardwareMap.get(ServoImplEx.class, "rightArm");
        grip = hardwareMap.get(ServoImplEx.class, "grip");
        colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "gripSensor");

        leftArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightArm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        colourRangeSensor.enableLed(true);

        leftArm.setDirection(ServoImplEx.Direction.REVERSE);
    }

    public sampleColour getSampleColour() {
        double red = colourRangeSensor.red();
        double green = colourRangeSensor.green();
        double blue = colourRangeSensor.blue();
        double distance = colourRangeSensor.getDistance(DistanceUnit.MM);

        if ((red > (blue * settings.colourThreshMultiplier) && red > (green * settings.colourThreshMultiplier)) && !Double.isNaN(distance)) {
            return sampleColour.RED;
        } else if ((blue > (red * settings.colourThreshMultiplier) && blue > (green * settings.colourThreshMultiplier)) && !Double.isNaN(distance)) {
            return sampleColour.BLUE;
        } else if ((red > (blue * settings.colourThreshMultiplier) && green > (blue * settings.colourThreshMultiplier)) && !Double.isNaN(distance)) {
            return sampleColour.YELLOW;
        } else {
            return sampleColour.NONE;
        }
    }

    //ARM
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

    public class depositArm implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftArm.setPosition(settings.armDeposit);
            rightArm.setPosition(settings.armDeposit);

            return false;
        }
    }

    public Action depositArm() {
        return new depositArm();
    }

    public class lowerArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftArm.setPosition(settings.armLowered);
            rightArm.setPosition(settings.armLowered);

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
