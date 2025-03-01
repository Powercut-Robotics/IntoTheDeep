package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.GBTorqueServo;
import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.RevServo;
import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.TCS34725;

@Config
public class Ancillary {
    public ServoImplEx intakeWheels;
    public GBTorqueServo intakeLeftArm = new GBTorqueServo(), intakeRightArm = new GBTorqueServo();
    public RevServo extendoLeft = new RevServo(), extendoRight = new RevServo(), upperLeftArm = new RevServo(), upperRightArm = new RevServo(), grip = new RevServo();
    public TCS34725 colourSensor = null;
    public TouchSensor trayTouchSensor;

    public boolean intakeActive = false;

    private enum extendoPos {
        FULL,
        HALF,
        CLEARANCE,
        TRAVEL,
        TRANSFER
    }

    private extendoPos currentExtendoPos;

    private enum intakeArmPos {
        LOWER,
        LOWER_SAFE,
        EXPEL,
        TRAVEL,
        TRANSFER
    }

    private intakeArmPos currentIntakeArm;
    public enum sampleColour {
        BLUE,
        RED,
        YELLOW,
        NONE
    }


    public class sampleStatus {
        sampleColour colour;

    }

    public sampleStatus status = new sampleStatus();
    public static double colourThreshMultiplier = 1.5;

    public static double intakeArmSafe = 0.95;
    public static double intakeArmTransfer = 1.0;
    public static double intakeArmExpel = 0.4;

    public static double intakeArmIntakeSafe = 0.325;
    public static double intakeArmIntake = 0.318;

    public static double extendoIntake = 0.23;
    public static double extendoHalf = 0.3675;
    public static double extendoClearance = 0.4;
    public static double extendoTravel = 0.49;
    public static double extendoTransfer = 0.505;


    public static double upperArmSampDeposit = 0.85;
    public static double upperArmSpecDeposit = 0.98;

    public static double upperArmTravel = 0.5;
    public static double upperArmIntake = 1.0;
    public static double upperArmTransfer = 0.0;

    public static double gripClosed = 0.5;
    public static double gripOpen = 0.3;



    protected void init(HardwareMap hardwareMap) {
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

        intakeLeftArmRaw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeRightArmRaw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoLeftRaw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoRightRaw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        upperLeftArmRaw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        upperRightArmRaw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        gripRaw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeWheels.setPwmRange(new PwmControl.PwmRange(500, 2500));

        upperLeftArmRaw.setDirection(ServoImplEx.Direction.REVERSE);
        intakeLeftArmRaw.setDirection(ServoImplEx.Direction.REVERSE);
        extendoLeftRaw.setDirection(Servo.Direction.REVERSE);

        intakeLeftArm.init(intakeLeftArmRaw);
        intakeRightArm.init(intakeRightArmRaw);

        extendoLeft.init(extendoLeftRaw);
        extendoRight.init(extendoRightRaw);
        upperLeftArm.init(upperLeftArmRaw);
        upperRightArm.init(upperRightArmRaw);

        grip.init(gripRaw);
    }

    public sampleColour getSampleColour() {
        double red = colourSensor.red();
        double green = colourSensor.green();
        double blue = colourSensor.blue();


        if ((red > (blue * colourThreshMultiplier) && red > (green * colourThreshMultiplier))) {
            status.colour = sampleColour.RED;
            return sampleColour.RED;
        } else if ((blue > (red * colourThreshMultiplier) && blue > (green * colourThreshMultiplier))) {
            status.colour = sampleColour.BLUE;
            return sampleColour.BLUE;
        } else if ((red > (blue * colourThreshMultiplier) && green > (blue * colourThreshMultiplier))) {
            status.colour = sampleColour.YELLOW;
            return sampleColour.YELLOW;
        } else {
            return sampleColour.NONE;
        }
    }

    //main system control
    //  public void setExtendo(double pos) {
    //      double limit = extendoTransfer - extendoIntake;
    //
    //      extendoLeft.setPosition(extendoIntake + (limit * pos));
    //      extendoRight.setPosition(extendoIntake + (limit * pos));
    //   }

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
                first = false;
                lastDetectedTime = System.currentTimeMillis();
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

            if (sample == sampleColour.NONE || !intakeActive) {
                // Check if 500 ms have passed since the last detection
                if ((System.currentTimeMillis() - lastDetectedTime > 700) || inTray || !intakeActive) {
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

    public Action transferAction() {
        return new TransferAction();
    }

    //Intake Extendo
    public class IntakeExtendo implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                currentExtendoPos = extendoPos.FULL;
                first = false;

                extendoLeft.setPosition(extendoIntake);
                extendoRight.setPosition(extendoIntake);
            }

            return extendoLeft.isMoving() && currentExtendoPos == extendoPos.FULL;
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
                currentExtendoPos = extendoPos.HALF;
                first = false;

                extendoLeft.setPosition(extendoHalf);
                extendoRight.setPosition(extendoHalf);
            }

            return extendoLeft.isMoving() && currentExtendoPos == extendoPos.HALF;
        }
    }

    public Action halfExtendo() {
        return new HalfExtendo();
    }

    public class ClearanceExtendo implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                currentExtendoPos = extendoPos.CLEARANCE;
                first = false;

                extendoLeft.setPosition(extendoClearance);
                extendoRight.setPosition(extendoClearance);
            }

            return extendoLeft.isMoving() && currentExtendoPos == extendoPos.CLEARANCE;
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
                currentExtendoPos = extendoPos.TRAVEL;
                first = false;

                extendoLeft.setPosition(extendoTravel);
                extendoRight.setPosition(extendoTravel);
            }

            return extendoLeft.isMoving() && currentExtendoPos == extendoPos.TRAVEL;
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
                currentExtendoPos = extendoPos.TRANSFER;
                first = false;

                extendoLeft.setPosition(extendoTransfer);
                extendoRight.setPosition(extendoTransfer);
            }

            return extendoLeft.isMoving() && currentExtendoPos == extendoPos.TRANSFER;
        }
    }

    public Action transferExtendo() {
        return new TransferExtendo();
    }

//Intake arm

    public class IntakeLowerArm implements Action {
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                currentIntakeArm = intakeArmPos.LOWER;
                first = false;

                intakeLeftArm.setPosition(intakeArmIntake);
                intakeRightArm.setPosition(intakeArmIntake);
            }
            return intakeLeftArm.isMoving() && currentIntakeArm == intakeArmPos.LOWER;
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
                currentIntakeArm = intakeArmPos.LOWER_SAFE;
                first = false;

                intakeLeftArm.setPosition(intakeArmIntakeSafe);
                intakeRightArm.setPosition(intakeArmIntakeSafe);
            }
            return intakeLeftArm.isMoving() && currentIntakeArm == intakeArmPos.LOWER_SAFE;
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
                currentIntakeArm = intakeArmPos.EXPEL;
                first = false;

                intakeLeftArm.setPosition(intakeArmExpel);
                intakeRightArm.setPosition(intakeArmExpel);
            }
            return intakeLeftArm.isMoving() && currentIntakeArm == intakeArmPos.EXPEL;
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
                currentIntakeArm = intakeArmPos.TRAVEL;
                first = false;

                intakeLeftArm.setPosition(intakeArmSafe);
                intakeRightArm.setPosition(intakeArmSafe);
            }

            return intakeLeftArm.isMoving() && currentIntakeArm == intakeArmPos.TRAVEL;
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
                currentIntakeArm = intakeArmPos.TRANSFER;
                first = false;

                intakeLeftArm.setPosition(intakeArmTransfer);
                intakeRightArm.setPosition(intakeArmTransfer);
            }

            return intakeLeftArm.isMoving() && currentIntakeArm == intakeArmPos.TRANSFER;
        }
    }

    public Action intakeTransferArm() {
        return new IntakeTransferArm();
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

    // Outtake Arm

    public class DepositSampArm implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                first = false;

                upperLeftArm.setPosition(upperArmSampDeposit);
                upperRightArm.setPosition(upperArmSampDeposit);
            }
            return !upperLeftArm.isMoving();
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
                first = false;

                upperLeftArm.setPosition(upperArmSpecDeposit);
                upperRightArm.setPosition(upperArmSpecDeposit);
            }
            return !upperLeftArm.isMoving();
        }
    }

    public Action depositSpecArm() {
        return new DepositSpecArm();
    }

    public class SpecIntakeArm implements Action {

        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                first = false;

                upperLeftArm.setPosition(upperArmIntake);
                upperRightArm.setPosition(upperArmIntake);
            }
            return !upperLeftArm.isMoving();
        }
    }

    public Action specIntakeArm() {
        return new SpecIntakeArm();
    }

    public class OuttakeTravelArm implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                first = false;

                upperLeftArm.setPosition(upperArmTravel);
                upperRightArm.setPosition(upperArmTravel);
            }
            return !upperLeftArm.isMoving();
        }
    }

    public Action outtakeTravelArm() {
        return new OuttakeTravelArm();
    }

    public class OuttakeTransferArm implements Action {
        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                first = false;

                upperLeftArm.setPosition(upperArmTransfer);
                upperRightArm.setPosition(upperArmTransfer);
            }

            return !upperLeftArm.isMoving();
        }
    }

    public Action outtakeTransferArm() {
        return new OuttakeTransferArm();
    }


    // GRIP
    public class CloseGrip implements Action {

        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                first = false;
                grip.setPosition(gripClosed);
            }

            return grip.isMoving();
        }
    }

    public Action closeGrip() {
        return new CloseGrip();
    }

    public class OpenGrip implements Action {

        private boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                first = false;

                grip.setPosition(gripOpen);
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
                first = false;

                grip.servo.setPwmDisable();
            }

            return false;
        }
    }

    public Action relaxGrip() {
        return new RelaxGrip();
    }

}