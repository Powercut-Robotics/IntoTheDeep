package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;

import java.util.ArrayList;
import java.util.List;


@Config
@TeleOp
public class LiftTuner extends OpMode {

    private final Robot robot = new Robot();

    private Lift lift;

    private LightSystem light;


    public static int pos = 0;

    public static PIDCoefficientsEx liftCoefficients = new PIDCoefficientsEx(0.0045, 0.00, 0.000, 500, 150, 0);
    private PIDEx liftPID = new PIDEx(liftCoefficients);


    private List<Action> ancillaryActions = new ArrayList<>();

    @Override
    public void init() {
        robot.init(hardwareMap);
        lift = robot.getLift();
        light = robot.getLight();

        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        if (lift.isLiftAvailable && lift.liftStop.getState()){
            telemetry.addLine("Lift Holding");
            lift.setLiftPower(0.08);
        }

        telemetry.addData("lift available",lift.isLiftAvailable);
        telemetry.addData("Lift stop", lift.liftStop.getState());

        lift.setLiftPower(liftPID.calculate(pos, (double) (lift.leftLift.getCurrentPosition() + lift.rightLift.getCurrentPosition()) / 2));

        light.partyWaves();

        telemetry.addData("Left Pos", lift.leftLift.getCurrentPosition());
        telemetry.addData("Right Pos", lift.leftLift.getCurrentPosition());
        telemetry.addData("Target", pos);

        telemetry.update();
    }
}
