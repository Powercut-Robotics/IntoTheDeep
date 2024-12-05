package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.powercut.hardware.Arm;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.settings;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class testing extends OpMode {

    private final Arm arm = new Arm();
    private final Lift lift = new Lift();
    private final Drivetrain drive = new Drivetrain();
    private final LightSystem light = new LightSystem();

    private boolean isActionRunning = false;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    @Override
    public void init() {
        arm.init(hardwareMap);
        lift.init(hardwareMap);
        drive.init(hardwareMap);
        light.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        light.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double theta = gamepad1.right_stick_x;
        double yaw = drive.getYaw();
        drive.setDrivetrainPowers(x, y, theta,1);
telemetry.addData("Theta", theta);
        telemetry.addData("Yaw:", yaw);
        telemetry.addData("Lift Pos", "%d, %d", lift.leftLift.getCurrentPosition(), lift.rightLift.getCurrentPosition());
        telemetry.addData("Lift Power", "%4.3f, %4.3f", lift.leftLift.getPower(), lift.rightLift.getPower());
        telemetry.addData("Colour Sensor Values (RGBA), Range", "%d, %d, %d, %d, %5.2f", arm.colourRangeSensor.red(), arm.colourRangeSensor.green(), arm.colourRangeSensor.blue(), arm.colourRangeSensor.alpha(), arm.colourRangeSensor.getDistance(DistanceUnit.MM));


        if (Math.abs(-gamepad2.right_stick_y) > 0.05) {
            isActionRunning = false;
            double liftPower = -gamepad2.right_stick_y;
            lift.setLiftPower(liftPower);
        } else if (!isActionRunning) {
            double liftPower = settings.liftHoldPower;
            lift.setLiftPower(liftPower);
        }

        if (!isActionRunning) {
            runningActions.clear();
        }
        if (gamepad1.dpad_up) {
            runningActions.add(arm.raiseArm());
        }
        if (gamepad1.dpad_right) {
            runningActions.add(arm.depositArm());
        }
        if (gamepad1.dpad_down) {
            runningActions.add(arm.lowerArm());
        }


//        if (gamepad1.dpad_up) {
//            runningActions.add(arm.raiseArm());
//        }
//
//        if (gamepad1.dpad_down) {
//            runningActions.add(arm.depositArm());
//        }
//
//        if (gamepad1.dpad_right) {
//            runningActions.add(arm.lowerArm());
//        }

        if (gamepad2.triangle) {
            runningActions.clear();
            isActionRunning = true;
            runningActions.add(new SequentialAction(lift.liftTopRung(),new InstantAction(() -> isActionRunning = false)));
        }

        if (gamepad2.circle) {
            runningActions.clear();
            isActionRunning = true;
            runningActions.add(new SequentialAction(lift.liftTopBasket(),new InstantAction(() -> isActionRunning = false)));
        }

        if (gamepad2.cross) {
            runningActions.clear();
            isActionRunning = true;
            runningActions.add(new SequentialAction(lift.liftRetract(),new InstantAction(() -> isActionRunning = false)));
        }

        if (gamepad1.left_bumper) {
            runningActions.add(arm.closeGrip());
        }

        if (gamepad1.right_bumper) {
            runningActions.add(arm.openGrip());
        }

        if (gamepad1.square || gamepad2.square) {
            runningActions.clear();
        }


        Arm.sampleColour sampleColour = arm.getSampleColour();
        String colour = "";
        if (sampleColour == Arm.sampleColour.RED) {
            light.red();
            colour = "Red";
        } else if (sampleColour == Arm.sampleColour.YELLOW) {
            light.yellow();
            colour = "Yellow";
        } else if (sampleColour == Arm.sampleColour.BLUE) {
            light.blue();
            colour = "Blue";
        } else {
            light.greyLarson();
            colour = "None";
        }

        telemetry.addLine("Sample: " + colour);


        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        telemetry.update();
    }
}
