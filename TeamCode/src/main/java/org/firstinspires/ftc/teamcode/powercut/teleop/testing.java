package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.powercut.hardware.Arm;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class testing extends OpMode {

    private final Arm arm = new Arm();
    private final Lift lift = new Lift();
    private final Drivetrain drive = new Drivetrain();
    private final LightSystem light = new LightSystem();

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    @Override
    public void init() {
        arm.init(hardwareMap);
        lift.init(hardwareMap);
        drive.init(hardwareMap);
        light.init(hardwareMap);


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
        double yawDegrees = drive.getYaw();
        double yaw = Math.toRadians(yawDegrees);

        double x_rotated = x * Math.cos(-yaw) - y * Math.sin(-yaw);
        double y_rotated = x * Math.sin(-yaw) + y * Math.cos(-yaw);
        drive.setDrivetrainPowers(x_rotated, y_rotated, theta,1);

        telemetry.addData("Yaw:", yaw);
        telemetry.addData("xRot, yRot", "%3.2f, %3.2f", x_rotated, y_rotated);
        telemetry.addData("LeftLift:", lift.leftLift.getCurrentPosition());
        telemetry.addData("RightLift:", lift.rightLift.getCurrentPosition());
        telemetry.addData("Colour Sensor Values (RGBA), Range", "%3.0f, %3.0f, %3.0f, %3.0f, %5.2f", arm.colourRangeSensor.red(), arm.colourRangeSensor.green(), arm.colourRangeSensor.blue(), arm.colourRangeSensor.alpha(), arm.colourRangeSensor.getDistance(DistanceUnit.MM));


        double liftPower = -gamepad2.right_stick_y;
        lift.setLiftPower(liftPower);


        if (gamepad1.dpad_up) {
            runningActions.add(arm.raiseArm());
        }

        if (gamepad1.dpad_down) {
            runningActions.add(arm.depositArm());
        }

        if (gamepad1.dpad_right) {
            runningActions.add(arm.lowerArm());
        }

        if (gamepad2.triangle) {
            runningActions.add(lift.liftTopRung());
        }

        if (gamepad2.circle) {
            runningActions.add(lift.liftTopBasket());
        }

        if (gamepad2.cross) {
            runningActions.add(lift.liftRetract());
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
            colour = "Green";
        } else if (sampleColour == Arm.sampleColour.BLUE) {
            light.red();
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
