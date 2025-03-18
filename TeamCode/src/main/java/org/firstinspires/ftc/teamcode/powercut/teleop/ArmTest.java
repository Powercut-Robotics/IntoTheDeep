package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;
import org.firstinspires.ftc.teamcode.powercut.hardware.SafeAncillary;


@Config
@TeleOp
public class ArmTest extends OpMode {

    private final Robot robot = new Robot();

    private SafeAncillary ancillary;

    private LightSystem light;
    private Drivetrain drive;

    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    public static double upperPos =0.50;
    public static double extendo = 0.5;
    public static double clawPos =0.50;
    public static double lowerPos =0.50;
    public static double wheelSpeed =0.50;

    @Override
    public void init() {
        robot.init(hardwareMap);
        ancillary = robot.getAncillary();
        light = robot.getLight();
        drive = robot.getDrive();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void loop() {
        ancillary.intakeLeftArm.setPosition(lowerPos);
        ancillary.intakeRightArm.setPosition(lowerPos);
        telemetry.addData("intakeLeftArm isMoving", ancillary.intakeLeftArm.isMoving());
        telemetry.addData("intakeRightArm isMoving", ancillary.intakeRightArm.isMoving());

        ancillary.upperLeftArm.setPosition(upperPos);
        ancillary.upperRightArm.setPosition(upperPos);
        telemetry.addData("upperLeftArm isMoving", ancillary.upperLeftArm.isMoving());
        telemetry.addData("upperRightArm isMoving", ancillary.upperRightArm.isMoving());

        ancillary.extendoLeft.setPosition(extendo);
        ancillary.extendoRight.setPosition(extendo);
        telemetry.addData("extendoLeft isMoving", ancillary.extendoLeft.isMoving());
        telemetry.addData("extendoRight isMoving", ancillary.extendoRight.isMoving());

        ancillary.grip.setPosition(clawPos);
        ancillary.intakeWheels.setPosition(wheelSpeed);

        light.partyWaves();

//        telemetry.addData("Colour", "%d,%d,%d", ancillary.colourSensor.red(), ancillary.colourSensor.green(), ancillary.colourSensor.blue());
//        telemetry.addData("US Reads LR", "%d, %d", drive.leftUpperUS.getDistance(), drive.rightUpperUS.getDistance());
//        telemetry.addData("ToF Reads LR", "%4.1f, %4.1f", drive.frontLeftToF.getDistance(DistanceUnit.MM), drive.frontRightToF.getDistance(DistanceUnit.MM));
        double leftLowerMVout = drive.leftLowerUS.getVoltage() * 1000;
        double rightLowerMVout = drive.rightLowerUS.getVoltage() * 1000;

        telemetry.addData("Analog", "%5.2f, %5.2f", (leftLowerMVout*520)/3300, (rightLowerMVout*520)/3300);

        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
