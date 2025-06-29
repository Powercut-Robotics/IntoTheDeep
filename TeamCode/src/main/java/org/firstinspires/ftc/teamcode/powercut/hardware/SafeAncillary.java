package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.GBSpeedServo;
import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.GBTorqueServo;
import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.RevServo;
import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.TCS34725;

@Config
public class SafeAncillary {
    public ServoImplEx intakeWheels;
    public GBTorqueServo extendoLeft = new GBTorqueServo(), extendoRight = new GBTorqueServo();

    public GBSpeedServo intakeLeftArm = new GBSpeedServo(), intakeRightArm = new GBSpeedServo(), upperLeftArm = new GBSpeedServo(), upperRightArm = new GBSpeedServo();
    public RevServo grip = new RevServo();
    public TCS34725 colourSensor = null;
    public TouchSensor trayTouchSensor;

    public enum sampleColour {
        BLUE,
        RED,
        YELLOW,
        NONE
    }

    public sampleColour currentColour;

    public static double colourThreshMultiplier = 1.5;

    public static double intakeArmSafe = 0.89;
    public static double intakeArmTransfer = 0.93;
    public static double intakeArmExpel = 0.35;

    public static double intakeArmIntakeSafe = 0.28;
    public static double intakeArmIntake = 0.265;

    public static double extendoIntake = 0.28;
    public static double extendoHalf = 0.39;

    public static double extendoSpecClearance = 0.4;
    public static double extendoClearance = 0.48;
    public static double extendoTravel = 0.5;
    public static double extendoTransfer = 0.548;


    public static double upperArmSampDeposit = 0.9;
    public static double upperArmSpecDeposit = 0.94;

    public static double upperArmHighSpecDeposit = 0.8;

    public static double upperArmAngledSpecAlign = 0.85;
    public static double upperArmAngledSpecDeposit = 0.96;

    public static double upperArmLowTravel = 0.4;
    public static double upperArmHighTravel = 0.675;
    public static double upperArmIntake = 0.97;
    public static double upperArmIntakeLow = 1;
    public static double upperArmTransfer = 0.08;

    public static double gripClosed = 0.84;
    public static double gripLoose = 0.79;
    public static double gripOpen = 0.63;

    public static double transferTime = 650;

    public boolean intakeActive = false;




    public void init(HardwareMap hardwareMap) {
        ServoImplEx intakeLeftArmRaw = hardwareMap.get(ServoImplEx.class, "intakeLeftArm");
        ServoImplEx intakeRightArmRaw = hardwareMap.get(ServoImplEx.class, "intakeRightArm");
        ServoImplEx extendoLeftRaw = hardwareMap.get(ServoImplEx.class, "extendoLeft");
        ServoImplEx extendoRightRaw = hardwareMap.get(ServoImplEx.class, "extendoRight");
        ServoImplEx upperLeftArmRaw = hardwareMap.get(ServoImplEx.class, "leftArm");
        ServoImplEx upperRightArmRaw = hardwareMap.get(ServoImplEx.class, "rightArm");
        ServoImplEx gripRaw = hardwareMap.get(ServoImplEx.class, "grip");

        intakeWheels = hardwareMap.get(ServoImplEx.class, "intakeWheels");
        colourSensor = hardwareMap.get(TCS34725.class, "intakeColour");
        trayTouchSensor = hardwareMap.get(TouchSensor.class, "trayTouch");

        intakeWheels.setPwmRange(new PwmControl.PwmRange(1000, 2000));

        upperLeftArmRaw.setDirection(ServoImplEx.Direction.REVERSE);
        intakeLeftArmRaw.setDirection(ServoImplEx.Direction.REVERSE);
        extendoLeftRaw.setDirection(ServoImplEx.Direction.REVERSE);

        intakeLeftArm.init(intakeLeftArmRaw);
        intakeRightArm.init(intakeRightArmRaw);
        extendoLeft.init(extendoLeftRaw);
        extendoRight.init(extendoRightRaw);
        upperLeftArm.init(upperLeftArmRaw);
        upperRightArm.init(upperRightArmRaw);

        grip.init(gripRaw);
    }

//    public boolean isUpperArmSafe() {
//        double position = upperLeftArm.getPosition();
//
//        return !(position > upperArmSafetyBounds[0] && position < upperArmSafetyBounds[1]);
//    }
//
//    public boolean isLowerArmSafe() {
//        double position = intakeLeftArm.getPosition();
//
//        return !(position > lowerArmSafetyBounds[0] && position < lowerArmSafetyBounds[1]);
//    }
//
//    public boolean isExtendoClear() {
//        return ((extendoLeft.getPosition() > extendoClearance) && !extendoLeft.isPositionIncreasing());
//    }
//
//
//    public boolean isSystemMoving() {
//        boolean intakeMoving = intakeLeftArm.isMoving();
//        boolean upperArmMoving = upperLeftArm.isMoving();
//        boolean extendoMoving = extendoLeft.isMoving();
//
//        return intakeMoving || upperArmMoving || extendoMoving;
//    }

    public void refreshSampleColour() {
        double red = colourSensor.red();
        double green = colourSensor.green();
        double blue = colourSensor.blue();

        if ((red > (blue * colourThreshMultiplier) && green > (blue * colourThreshMultiplier))) {
            currentColour = sampleColour.YELLOW;
        } else if ((red > (blue * colourThreshMultiplier) && red > (green * colourThreshMultiplier))) {
            currentColour = sampleColour.RED;
        } else if ((blue > (red * colourThreshMultiplier) && blue > (green * colourThreshMultiplier))) {
            currentColour = sampleColour.BLUE;
        } else {
            currentColour = sampleColour.NONE;
        }
    }
    public sampleColour getSampleColour() {
        double red = colourSensor.red();
        double green = colourSensor.green();
        double blue = colourSensor.blue();

        if ((red > (blue * colourThreshMultiplier) && green > (blue * colourThreshMultiplier))) {
            currentColour = sampleColour.YELLOW;
            return sampleColour.YELLOW;
        } else if ((red > (blue * colourThreshMultiplier) && red > (green * colourThreshMultiplier))) {
            currentColour = sampleColour.RED;
            return sampleColour.RED;
        } else if ((blue > (red * colourThreshMultiplier) && blue > (green * colourThreshMultiplier))) {
            currentColour = sampleColour.BLUE;
            return sampleColour.BLUE;
        } else {
            currentColour = sampleColour.NONE;
            return sampleColour.NONE;
        }
    }

    public void relaxSystem() {
        intakeActive = false;
        intakeWheels.setPosition(0.5);
        intakeLeftArm.servo.setPwmDisable();
        intakeRightArm.servo.setPwmDisable();
        intakeWheels.setPwmDisable();
        extendoLeft.servo.setPwmDisable();
        extendoRight.servo.setPwmDisable();
        grip.servo.setPwmDisable();
        upperLeftArm.servo.setPwmDisable();
        upperRightArm.servo.setPwmDisable();

    }
    public Action relaxSystemAction() {
        return new InstantAction(() -> relaxSystem());
    }
    public void engageSystem() {
        intakeLeftArm.servo.setPwmEnable();
        intakeRightArm.servo.setPwmEnable();
        intakeWheels.setPwmEnable();
        extendoLeft.servo.setPwmEnable();
        extendoRight.servo.setPwmEnable();
        grip.servo.setPwmEnable();
        upperLeftArm.servo.setPwmEnable();
        upperRightArm.servo.setPwmEnable();
    }

    public Action engageAction() {
        return new InstantAction(() -> engageSystem());
    }

    //Main Actions
    public class IntakeAction implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                intakeActive = true;
                first = false;
            }

            sampleColour sample = getSampleColour();

            if (sample != sampleColour.NONE || !intakeActive) {
                intakeWheels.setPosition(0.5);
                intakeActive = false;
                return false;
            } else {
                intakeWheels.setPosition(0);
                return true;
            }
        }
    }

    public Action intakeAction() {
        return new IntakeAction();
    }

    public class ExpelAction implements Action {
        private long lastDetectedTime = System.currentTimeMillis(); // Tracks the last time a sample was detected
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                intakeActive = true;
                lastDetectedTime = System.currentTimeMillis();

                first = false;
            }

            sampleColour sample = getSampleColour();
            intakeWheels.setPosition(1);

            if (sample == sampleColour.NONE || !intakeActive) {
                if (!intakeActive) {
                    intakeWheels.setPosition(0.5);
                    return false; // Stop returning true
                } else if ((System.currentTimeMillis() - lastDetectedTime) > 1000) {
                    intakeActive = false;
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

    public class TransferAction implements Action {

        private long lastDetectedTime = System.currentTimeMillis(); // Tracks the last time a sample was detected

        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                intakeActive = true;
                first = false;
                lastDetectedTime = System.currentTimeMillis();
            }

            intakeWheels.setPosition(1);
            sampleColour sample = getSampleColour();
            boolean inTray = trayTouchSensor.isPressed();

            //if (sample == sampleColour.NONE || !intakeActive) {
                // Check if 500 ms have passed since the last detection
                if ((System.currentTimeMillis() - lastDetectedTime > transferTime) || inTray || !intakeActive) {
                    intakeActive = false;
                    intakeWheels.setPosition(0.5);
                    return false; // Stop returning true
                } else {
                    return true; // Continue returning true until timeout expires
                }
//            } else {
//                intakeWheels.setPosition(1);
//
//                // Update the last detected time when a sample is found
//                lastDetectedTime = System.currentTimeMillis();
//                return true;
//            }
        }
    }

    public Action transferAction() {
        return new TransferAction();
    }

    //Intake Extendo
    public class IntakeExtendo implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                extendoLeft.setPosition(extendoIntake);
                extendoRight.setPosition(extendoIntake);

                first = false;
            }

            return extendoLeft.isMoving();
        }
    }

    public Action intakeExtendo() {
        return new IntakeExtendo();
    }

    public class HalfExtendo implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                extendoLeft.setPosition(extendoHalf);
                extendoRight.setPosition(extendoHalf);

                first = false;
            }

            return extendoLeft.isMoving();
        }
    }

    public Action halfExtendo() {
        return new HalfExtendo();
    }

    public class ClearanceSpecExtendo implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                extendoLeft.setPosition(extendoSpecClearance);
                extendoRight.setPosition(extendoSpecClearance);
                first = false;
            }

            return extendoLeft.isMoving();
        }
    }

    public Action clearanceSpecExtendo() {
        return new ClearanceSpecExtendo();
    }


    public class ClearanceExtendo implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                extendoLeft.setPosition(extendoClearance);
                extendoRight.setPosition(extendoClearance);
                first = false;
            }

            return extendoLeft.isMoving();
        }
    }

    public Action clearanceExtendo() {
        return new ClearanceExtendo();
    }

    public class TravelExtendo implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                extendoLeft.setPosition(extendoTravel);
                extendoRight.setPosition(extendoTravel);

                first = false;
            }

            return extendoLeft.isMoving();
        }
    }

    public Action travelExtendo() {
        return new TravelExtendo();
    }

    public class TransferExtendo implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {

                extendoLeft.setPosition(extendoTransfer);
                extendoRight.setPosition(extendoTransfer);

                first = false;
            }

            return extendoLeft.isMoving();
        }
    }

    public Action transferExtendo() {
        return new TransferExtendo();
    }

// Arms
    public class IntakeLowerArm implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                intakeLeftArm.setPosition(intakeArmIntake);
                intakeRightArm.setPosition(intakeArmIntake);

                first = false;
            }
            return intakeLeftArm.isMoving();
        }
    }

    public Action intakeLowerArm() {
        return new IntakeLowerArm();
    }

    public class IntakeLowerArmSafe implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                intakeLeftArm.setPosition(intakeArmIntakeSafe);
                intakeRightArm.setPosition(intakeArmIntakeSafe);

                first = false;
            }
            return intakeLeftArm.isMoving();
        }
    }

    public Action intakeLowerArmSafe() {
        return new IntakeLowerArmSafe();
    }

    public class IntakeExpelArm implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                intakeLeftArm.setPosition(intakeArmExpel);
                intakeRightArm.setPosition(intakeArmExpel);

                first = false;
            }
            return intakeLeftArm.isMoving();
        }
    }

    public Action intakeExpelArm() {
        return new IntakeExpelArm();
    }

    public class IntakeTravelArm implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                intakeLeftArm.setPosition(intakeArmSafe);
                intakeRightArm.setPosition(intakeArmSafe);

                first = false;
            }

            return intakeLeftArm.isMoving();
        }
    }

    public Action intakeTravelArm() {
        return new IntakeTravelArm();
    }

    public class IntakeTransferArm implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                intakeLeftArm.setPosition(intakeArmTransfer);
                intakeRightArm.setPosition(intakeArmTransfer);

                first = false;
            }

            return intakeLeftArm.isMoving();
        }
    }

    public Action intakeTransferArm() {
        return new IntakeTransferArm();
    }

    // Outtake Arm

    public class DepositSampArm implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                upperLeftArm.setPosition(upperArmSampDeposit);
                upperRightArm.setPosition(upperArmSampDeposit);

                first = false;
            }

            return upperLeftArm.isMoving();
        }
    }

    public Action depositSampArm() {
        return new DepositSampArm();
    }

    public class DepositSpecArm implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                upperLeftArm.setPosition(upperArmSpecDeposit);
                upperRightArm.setPosition(upperArmSpecDeposit);

                first = false;
            }

            return upperLeftArm.isMoving();
        }
    }

    public Action depositSpecArm() {
        return new DepositSpecArm();
    }

    public class DepositMedianSpecArm implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                upperLeftArm.setPosition(upperArmAngledSpecDeposit);
                upperRightArm.setPosition(upperArmAngledSpecDeposit);

                first = false;
            }

            return upperLeftArm.isMoving();
        }
    }

    public Action depositMedianSpecArm() {
        return new DepositMedianSpecArm();
    }

    public class DepositHigherSpecArm implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                upperLeftArm.setPosition(upperArmHighSpecDeposit);
                upperRightArm.setPosition(upperArmHighSpecDeposit);

                first = false;
            }

            return upperLeftArm.isMoving();
        }
    }

    public Action depositHigherSpecArm() {
        return new DepositHigherSpecArm();
    }

    public class AlignMedianSpecArm implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                upperLeftArm.setPosition(upperArmAngledSpecAlign);
                upperRightArm.setPosition(upperArmAngledSpecAlign);

                first = false;
            }

            return upperLeftArm.isMoving();
        }
    }

    public Action alignMedianSpecArm() {
        return new AlignMedianSpecArm();
    }



    public class SpecIntakeArm implements Action {

        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                upperLeftArm.setPosition(upperArmIntake);
                upperRightArm.setPosition(upperArmIntake);

                first = false;
            }

            return upperLeftArm.isMoving();
        }
    }

    public Action specIntakeArm() {
        return new SpecIntakeArm();
    }

    public class LowSpecIntakeArm implements Action {

        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                upperLeftArm.setPosition(upperArmIntakeLow);
                upperRightArm.setPosition(upperArmIntakeLow);

                first = false;
            }

            return upperLeftArm.isMoving();
        }
    }

    public Action lowSpecIntakeArm() {
        return new LowSpecIntakeArm();
    }

    public class OuttakeLowerTravelArm implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                upperLeftArm.setPosition(upperArmLowTravel);
                upperRightArm.setPosition(upperArmLowTravel);

                first = false;
            }
            return upperLeftArm.isMoving();
        }
    }

    public Action outtakeLowerTravelArm() {
        return new OuttakeLowerTravelArm();
    }

    public class OuttakeHigherTravelArm implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                upperLeftArm.setPosition(upperArmHighTravel);
                upperRightArm.setPosition(upperArmHighTravel);

                first = false;
            }
            return upperLeftArm.isMoving();
        }
    }

    public Action outtakeHigherTravelArm() {
        return new OuttakeHigherTravelArm();
    }

    public class OuttakeTransferArm implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                upperLeftArm.setPosition(upperArmTransfer);
                upperRightArm.setPosition(upperArmTransfer);

                first = false;
            }

            return upperLeftArm.isMoving();
        }
    }

    public Action outtakeTransferArm() {
        return new OuttakeTransferArm();
    }


    //Intake Wheel
    public class SpinUpAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeActive = true;
            intakeWheels.setPosition(0);
            return false;

        }
    }

    public Action spinUpAction() {
        return new SpinUpAction();
    }

    public class HoldAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeWheels.setPosition(0.3);
            return false;

        }
    }

    public Action holdAction() {
        return new HoldAction();
    }

    public class SpinOutAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeActive = true;
            intakeWheels.setPosition(1);
            return false;

        }
    }

    public Action spinOutAction() {
        return new SpinOutAction();
    }

    public class WheelHaltAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeActive = false;
            intakeWheels.setPosition(0.5);

            return false;
        }
    }

    public Action wheelHaltAction() {
        return new WheelHaltAction();
    }


    // GRIP
    public class CloseGrip implements Action {

        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                grip.setPosition(gripClosed);

                first = false;
            }

            return grip.isMoving();
        }
    }

    public Action closeGrip() {
        return new CloseGrip();
    }

    public class LooseCloseGrip implements Action {

        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                grip.setPosition(gripLoose);

                first = false;
            }

            return grip.isMoving();
        }
    }

    public Action looseCloseGrip() {
        return new LooseCloseGrip();
    }

    public class OpenGrip implements Action {

        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                grip.setPosition(gripOpen);

                first = false;
            }

            return grip.isMoving();
        }
    }

    public Action openGrip() {
        return new OpenGrip();
    }

    public class RelaxGrip implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                grip.servo.setPwmDisable();

                first = false;
            }

            return false;
        }
    }

    public Action relaxGrip() {
        return new RelaxGrip();
    }

}
