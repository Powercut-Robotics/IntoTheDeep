package org.firstinspires.ftc.teamcode.powercut.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.GBTorqueServo;
import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.RevServo;
import org.firstinspires.ftc.teamcode.powercut.hardware.drivers.TCS34725;

@Config
public class SafeAncillary {
    private Robot robot = new Robot();
    private Lift lift;
    public ServoImplEx intakeWheels;
    public GBTorqueServo intakeLeftArm = new GBTorqueServo(), intakeRightArm = new GBTorqueServo();
    public RevServo extendoLeft = new RevServo(), extendoRight = new RevServo(), upperLeftArm = new RevServo(), upperRightArm = new RevServo(), grip = new RevServo();
    public TCS34725 colourSensor = null;
    public TouchSensor trayTouchSensor;

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


    private static final double[] upperArmSafetyBounds = {0.05, 0.33};

    private static final double[] lowerArmSafetyBounds = {0.6, 1.01};

    public enum SystemStates {
        HOME,
        INTAKE_FULL,
        INTAKE_HALF,
        INTAKE_NONE,
        EXPEL_FULL,
        EXPEL_NONE,
        TRANSFER,
        SPEC_IN_USE,
        SPEC_TRAVEL,
        SAMPLE_DEPOSIT
    }

    public SystemStates systemPose;

    public enum ExtendoPositions {
        FULL,
        HALF,
        TRAVEL,
        IN
    }

    public ExtendoPositions extendoPose;

    public enum IntakePositions {
        INTAKE,
        INTAKE_SAFE,
        EXPEL,
        TRAVEL,
        TRANSFER
    }

    public IntakePositions intakePose;

    public enum OuttakePositions {
        TRANSFER,
        TRAVEL,
        DEPOSIT,
        INTAKE
    }

    public OuttakePositions outtakePose;


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

        lift = robot.getLift();
    }

    public boolean isUpperArmSafe() {
        double position = upperLeftArm.getPosition();

        return !(position > upperArmSafetyBounds[0] && position < upperArmSafetyBounds[1]);
    }

    public boolean isLowerArmSafe() {
        double position = intakeLeftArm.getPosition();

        return !(position > lowerArmSafetyBounds[0] && position < lowerArmSafetyBounds[1]);
    }

    public boolean isExtendoClear() {
        return ((extendoLeft.getPosition() > extendoClearance) && !extendoLeft.isPositionIncreasing());
    }


    public boolean isSystemMoving() {
        boolean intakeMoving = intakeLeftArm.isMoving();
        boolean upperArmMoving = upperLeftArm.isMoving();
        boolean extendoMoving = extendoLeft.isMoving();

        return intakeMoving || upperArmMoving || extendoMoving;
    }


}
