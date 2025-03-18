package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;

import java.util.ArrayList;
import java.util.List;


@Config
@TeleOp
public class LiftTest extends OpMode {

    private final Robot robot = new Robot();

    private Lift lift;

    private LightSystem light;
    private Drivetrain drive;

    public static double upperPos =0.50;
    public static double extendo = 0.5;
    public static double clawPos =0.50;
    public static double lowerPos =0.50;
    public static double wheelSpeed =0.50;

    private List<Action> ancillaryActions = new ArrayList<>();

    @Override
    public void init() {
        robot.init(hardwareMap);
        lift = robot.getLift();
        light = robot.getLight();
        drive = robot.getDrive();

        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        if (lift.isLiftAvailable && !lift.liftStop.getState()){
            lift.setLiftPower(0.05);
        }

        if (gamepad1.cross) {
            ancillaryActions.clear();
            lift.kill();
                ancillaryActions.add(lift.liftRetract());
        }

        if (gamepad1.triangle) {
            ancillaryActions.clear();
            lift.kill();
            ancillaryActions.add(lift.liftTopBasket());
        }

        light.partyWaves();

        List<Action> newActions = new ArrayList<>();
        for (Action action : ancillaryActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        ancillaryActions = newActions;
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
