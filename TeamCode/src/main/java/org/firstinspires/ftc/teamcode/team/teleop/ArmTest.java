package org.firstinspires.ftc.teamcode.team.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team.hardware.Ancillary;
import org.firstinspires.ftc.teamcode.team.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.team.hardware.LightSystem;

@Disabled
@Config
@TeleOp
public class ArmTest extends OpMode {

    private final Ancillary ancillary = new Ancillary();

    private final LightSystem light = new LightSystem();
    private final Drivetrain drive = new Drivetrain();

    public static double upperPos =0.50;
    public static double extendo = 0.5;
    public static double clawPos =0.50;
    public static double lowerPos =0.50;
    public static double wheelSpeed =0.50;

    @Override
    public void init() {
        ancillary.init(hardwareMap);

        light.init(hardwareMap);
        drive.init(hardwareMap);


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

        light.partyWaves();

//        telemetry.addData("Colour", "%d,%d,%d", ancillary.colourSensor.red(), ancillary.colourSensor.green(), ancillary.colourSensor.blue());
//        telemetry.addData("US Reads LR", "%d, %d", drive.leftUpperUS.getDistance(), drive.rightUpperUS.getDistance());
//        telemetry.addData("ToF Reads LR", "%4.1f, %4.1f", drive.frontLeftToF.getDistance(DistanceUnit.MM), drive.frontRightToF.getDistance(DistanceUnit.MM));
//        double leftLowerMVout = drive.leftLowerUS.getVoltage() * 1000;
//        double rightLowerMVout = drive.rightLowerUS.getVoltage() * 1000;
//
//        telemetry.addData("Analog", "%5.2f, %5.2f", (leftLowerMVout*520)/3300, (rightLowerMVout*520)/3300);
        telemetry.update();
    }
}
