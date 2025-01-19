package org.firstinspires.ftc.teamcode.powercut.teleop;

import static org.firstinspires.ftc.teamcode.powercut.settings.extendoIntake;
import static org.firstinspires.ftc.teamcode.powercut.settings.extendoTransfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powercut.hardware.Intake;

@Config
@TeleOp
public class ExtendoTest extends OpMode {

    private final Intake intake = new Intake();


    public static double pos =0.50;


    @Override
    public void init() {
        intake.init(hardwareMap);




        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.cross) {
            intake.extendoLeft.setPosition(extendoIntake);
            intake.extendoRight.setPosition(extendoIntake);
        } else {
            intake.extendoLeft.setPosition(extendoTransfer);
            intake.extendoRight.setPosition(extendoTransfer);
        }
        telemetry.addData("pos", pos);



        telemetry.update();
    }
}
