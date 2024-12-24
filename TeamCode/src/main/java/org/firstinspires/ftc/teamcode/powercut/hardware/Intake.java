package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.powercut.settings;

import java.util.List;

public class Intake {
    private ServoImplEx lowerLeftArm, lowerRightArm, intakeWheels;
    public ColorRangeSensor colourRangeSensor = null;
    public TouchSensor trayTouchSensor;

    public enum sampleColour {
        BLUE,
        RED,
        YELLOW,
        NONE
    }

    public void init(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        lowerLeftArm = hardwareMap.get(ServoImplEx.class, "upperLeftArm");
        lowerRightArm = hardwareMap.get(ServoImplEx.class, "upperRightArm");
        intakeWheels = hardwareMap.get(ServoImplEx.class, "intakeWheels");
        colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "intakeColourSensor");
        trayTouchSensor = hardwareMap.get(TouchSensor.class, "trayTouchSensor");

        colourRangeSensor.enableLed(true);

        lowerLeftArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        lowerRightArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeWheels.setPwmRange(new PwmControl.PwmRange(500, 2500));


        lowerLeftArm.setDirection(ServoImplEx.Direction.REVERSE);
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

    public class raiseArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            lowerLeftArm.setPosition(settings.lowerArmRaised);
            lowerRightArm.setPosition(settings.lowerArmRaised);

            return false;
        }
    }

    public Action raiseArm() {
        return new raiseArm();
    }

    public class lowerArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            lowerLeftArm.setPosition(settings.lowerArmLowered);
            lowerRightArm.setPosition(settings.lowerArmLowered);

            return false;
        }
    }

    public Action lowerArm() {
        return new lowerArm();
    }

    public class intake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sampleColour sample = getSampleColour();

            if (sample == sampleColour.NONE) {
                intakeWheels.setPosition(1);
                return true;
            } else {
                intakeWheels.setPosition(0.5);
                return false;
            }

        }
    }
    public Action intake() {
        return new intake();
    }

    public class expel implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sampleColour sample = getSampleColour();

            if (sample == sampleColour.NONE) {
                intakeWheels.setPosition(0.5);
                return false;
            } else {
                intakeWheels.setPosition(0);
                return true;
            }

        }
    }
    public Action expel() {
        return new expel();
    }

    public class transfer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sampleColour sample = getSampleColour();
            boolean inTray = trayTouchSensor.isPressed();

            if (inTray) {
                intakeWheels.setPosition(0.5);
                return false;
            } else {
                intakeWheels.setPosition(0);
                return true;
            }

        }
    }
    public Action transfer() {
        return new transfer();
    }
}
