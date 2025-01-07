package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.powercut.settings;

import java.util.List;

public class Intake {
    private ServoImplEx intakeLeftArm, intakeRightArm, extendoLeft, extendoRight, intakeWheels;
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

        intakeLeftArm = hardwareMap.get(ServoImplEx.class, "intakeLeftArm");
        intakeRightArm = hardwareMap.get(ServoImplEx.class, "intakeRightArm");
        extendoLeft = hardwareMap.get(ServoImplEx.class, "extendoRight");
        extendoRight = hardwareMap.get(ServoImplEx.class, "extendoRight");
        intakeWheels = hardwareMap.get(ServoImplEx.class, "intakeWheels");
        colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "intakeColour");
        trayTouchSensor = hardwareMap.get(TouchSensor.class, "trayTouch");

        colourRangeSensor.enableLed(true);

        intakeLeftArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeRightArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeWheels.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoRight.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intakeLeftArm.setDirection(ServoImplEx.Direction.REVERSE);
        extendoLeft.setDirection(Servo.Direction.REVERSE);
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

    // ARM
    public class IntakeExtendo implements Action {
        private long startTime;
        private static final long DURATION = 300;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            extendoLeft.setPosition(settings.extendoIntake);
            extendoRight.setPosition(settings.extendoIntake);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action intakeExtendo() {
        return new IntakeExtendo();
    }

    public class TransferExtendo implements Action {
        private long startTime;
        private static final long DURATION = 300;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            extendoLeft.setPosition(settings.extendoTransfer);
            extendoRight.setPosition(settings.extendoTransfer);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action transferExtendo() {
        return new TransferExtendo();
    }

    public class RaiseArm implements Action {
        private long startTime;
        private static final long DURATION = 300;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            intakeLeftArm.setPosition(settings.intakeArmTransfer);
            intakeRightArm.setPosition(settings.intakeArmTransfer);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action raiseArm() {
        return new RaiseArm();
    }

    public class LowerArm implements Action {
        private long startTime;
        private static final long DURATION = 300;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            intakeLeftArm.setPosition(settings.intakeArmIntake);
            intakeRightArm.setPosition(settings.intakeArmIntake);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action lowerArm() {
        return new LowerArm();
    }

    // INTAKE
    public class IntakeAction implements Action {
        private long startTime;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

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
        return new IntakeAction();
    }

    // EXPEL
    public class ExpelAction implements Action {
        private long startTime;
        private static final long DURATION = 300;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

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
        return new ExpelAction();
    }

    // TRANSFER
    public class TransferAction implements Action {
        private long startTime;
        private static final long DURATION = 300;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

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
        return new TransferAction();
    }
}
