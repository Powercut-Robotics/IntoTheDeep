package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Intake;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Outtake;

@Config
@TeleOp
public class ArmTest extends OpMode {

    private final Intake intake = new Intake();
    private final Outtake outtake = new Outtake();
    private final LightSystem light = new LightSystem();
    private final Drivetrain drive = new Drivetrain();

    public static double upperPos =0.50;
    public static double clawPos =0.50;
    public static double lowerPos =0.50;
    public static double wheelSpeed =0.50;

    @Override
    public void init() {
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        light.init(hardwareMap);
        drive.init(hardwareMap);


        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void loop() {
        intake.intakeLeftArm.setPosition(lowerPos);
        intake.intakeRightArm.setPosition(lowerPos);

        outtake.leftArm.setPosition(upperPos);
        outtake.rightArm.setPosition(upperPos);

        outtake.grip.setPosition(clawPos);

        intake.intakeWheels.setPosition(wheelSpeed);
        light.greyLarson();

        telemetry.addData("Colour", "%d,%d,%d", intake.colourSensor.red(), intake.colourSensor.green(), intake.colourSensor.blue());
        telemetry.addData("US Reads LR", "%d, %d", drive.leftUpperUS.getDistance(), drive.rightUpperUS.getDistance());
        telemetry.addData("ToF Reads LR", "%4.1f, %4.1f", drive.frontLeftToF.getDistance(DistanceUnit.MM), drive.frontRightToF.getDistance(DistanceUnit.MM));
        double leftLowerMVout = drive.leftLowerUS.getVoltage() * 1000;
        double rightLowerMVout = drive.rightLowerUS.getVoltage() * 1000;

        telemetry.addData("Analog", "%5.2f, %5.2f", (leftLowerMVout*520)/3300, (rightLowerMVout*520)/3300);
        telemetry.update();
    }
}
