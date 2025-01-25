package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;

@Config
@TeleOp
public class USTest extends OpMode {

    private final Drivetrain drive = new Drivetrain();


    public static int gain = 0x01;
    public static int integration = 0x01;


    @Override
    public void init() {
        drive.init(hardwareMap);
        drive.imu.resetYaw();



        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void loop() {

        double yaw = drive.getYaw();
        telemetry.addData("Yaw:", yaw);
//        telemetry.addData("US Reads LR", "%d, %d", drive.leftUpperUS.getDistance(), drive.rightUpperUS.getDistance());
//        telemetry.addData("ToF Reads LR", "%4.1f, %4.1f", drive.frontLeftToF.getDistance(DistanceUnit.MM), drive.frontRightToF.getDistance(DistanceUnit.MM));
        double leftLowerMVout = drive.leftLowerUS.getVoltage() * 1000;
        double rightLowerMVout = drive.rightLowerUS.getVoltage() * 1000;


//        telemetry.addData("Analog", "%5.2f, %5.2f", leftLowerMVout , rightLowerMVout);
        telemetry.addData("Analog", "%5.2f, %5.2f", (leftLowerMVout*520)/3300, (rightLowerMVout*520)/3300);




        telemetry.update();
    }
}
