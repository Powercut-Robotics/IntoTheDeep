package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.powercut.settings;

public class Intake {
    public ServoImplEx intakeLeftArm, intakeRightArm, extendoLeft, extendoRight, intakeWheels;
    public ColorSensor colourSensor = null;
    public TouchSensor trayTouchSensor;

    public enum sampleColour {
        BLUE,
        RED,
        YELLOW,
        NONE
    }

    public void init(HardwareMap hardwareMap) {
        intakeLeftArm = hardwareMap.get(ServoImplEx.class, "intakeLeftArm");
        intakeRightArm = hardwareMap.get(ServoImplEx.class, "intakeRightArm");
        extendoLeft = hardwareMap.get(ServoImplEx.class, "extendoLeft");
        extendoRight = hardwareMap.get(ServoImplEx.class, "extendoRight");
        intakeWheels = hardwareMap.get(ServoImplEx.class, "intakeWheels");
        colourSensor = hardwareMap.get(ColorSensor.class, "intakeColour");
        trayTouchSensor = hardwareMap.get(TouchSensor.class, "trayTouch");

        colourSensor.enableLed(true);

        intakeLeftArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeRightArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeWheels.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoRight.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intakeLeftArm.setDirection(ServoImplEx.Direction.REVERSE);
        extendoLeft.setDirection(Servo.Direction.REVERSE);
    }

    public sampleColour getSampleColour() {
        double red = colourSensor.red();
        double green = colourSensor.green();
        double blue = colourSensor.blue();


        if ((red > (blue * settings.colourThreshMultiplier) && red > (green * settings.colourThreshMultiplier))) {
            return sampleColour.RED;
        } else if ((blue > (red * settings.colourThreshMultiplier) && blue > (green * settings.colourThreshMultiplier))) {
            return sampleColour.BLUE;
        } else if ((red > (blue * settings.colourThreshMultiplier) && green > (blue * settings.colourThreshMultiplier))) {
            return sampleColour.YELLOW;
        } else {
            return sampleColour.NONE;
        }
    }

    public void setExtendo(double pos) {
        double limit = settings.extendoTransfer - settings.extendoIntake;

        extendoLeft.setPosition(settings.extendoIntake + (limit * pos));
        extendoRight.setPosition(settings.extendoIntake + (limit * pos));
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

    public class Transfer1Extendo implements Action {
        private long startTime;
        private static final long DURATION = 300;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            extendoLeft.setPosition(settings.extendoTravel);
            extendoRight.setPosition(settings.extendoTravel);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action transfer1Extendo() {
        return new Transfer1Extendo();
    }

    public class Transfer2Extendo implements Action {
        private long startTime;
        private static final long DURATION = 100;

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

    public Action transfer2Extendo() {
        return new Transfer2Extendo();
    }

    public class ClearanceExtendo implements Action {
        private long startTime;
        private static final long DURATION = 100;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            extendoLeft.setPosition(settings.extendoClearance);
            extendoRight.setPosition(settings.extendoClearance);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action clearanceExtendo() {
        return new ClearanceExtendo();
    }

    public class TravelArm implements Action {
        private long startTime;
        private static final long DURATION = 1100;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            intakeLeftArm.setPosition(settings.intakeArmSafe);
            intakeRightArm.setPosition(settings.intakeArmSafe);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action travelArm() {
        return new TravelArm();
    }

    public class TransferArm implements Action {
        private long startTime;
        private static final long DURATION = 200;

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

    public Action transferArm() {
        return new TransferArm();
    }

    public class LowerArm implements Action {
        private long startTime;
        private static final long DURATION = 1200;

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

    public class LowerArmSafe implements Action {
        private long startTime;
        private static final long DURATION = 1200;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            intakeLeftArm.setPosition(settings.intakeArmIntakeSafe);
            intakeRightArm.setPosition(settings.intakeArmIntakeSafe);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action lowerArmSafe() {
        return new LowerArmSafe();
    }

    // INTAKE
    public class IntakeAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            sampleColour sample = getSampleColour();

            if (sample == sampleColour.NONE) {
                intakeWheels.setPosition(0);
                return true;
            }

            intakeWheels.setPosition(0.5);
            return false;

        }
    }

    public Action intakeAction() {
        return new IntakeAction();
    }


    public class SpinUpAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeWheels.setPosition(0.);
            return false;

        }
    }

    public Action spinUpAction() {
        return new SpinUpAction();
    }

    // EXPEL

    public class ExpelAction implements Action {
        private long lastDetectedTime = System.currentTimeMillis(); // Tracks the last time a sample was detected

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sampleColour sample = getSampleColour();

            if (sample == sampleColour.NONE) {


                // Check if 500 ms have passed since the last detection
                if (System.currentTimeMillis() - lastDetectedTime > 1000) {
                    intakeWheels.setPosition(0.5);
                    return false; // Stop returning true
                } else {
                    return true; // Continue returning true until timeout expires
                }
            } else {
                intakeWheels.setPosition(1);

                // Update the last detected time when a sample is found
                lastDetectedTime = System.currentTimeMillis();
                return true;
            }
        }
    }

    public Action expelAction() {
        return new ExpelAction();
    }


    // TRANSFER
    public class TransferAction implements Action {

        private long lastDetectedTime = System.currentTimeMillis(); // Tracks the last time a sample was detected

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sampleColour sample = getSampleColour();
            boolean inTray = trayTouchSensor.isPressed();

            if (sample == sampleColour.NONE) {
                // Check if 500 ms have passed since the last detection
                if ((System.currentTimeMillis() - lastDetectedTime > 700) || inTray) {
                    intakeWheels.setPosition(0.5);
                    return false; // Stop returning true
                } else {
                    return true; // Continue returning true until timeout expires
                }
            } else {
                intakeWheels.setPosition(1);

                // Update the last detected time when a sample is found
                lastDetectedTime = System.currentTimeMillis();
                return true;
            }
        }
    }

    public Action transferAction() {
        return new TransferAction();
    }
}
