package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.powercut.hardware.Arm;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;

import java.util.ArrayList;
import java.util.List;

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
        if (gamepad1.right_bumper) {
            drive.setDrivetrainPowers(1,0,0,1);
        }

        if (gamepad1.cross) {
            lift.leftLift.setPower(0.25);
        }
        if (gamepad1.circle) {
            lift.leftLift.setPower(-0.25);
        }
        if (gamepad1.triangle) {
            lift.rightLift.setPower(0.25);
        }
        if (gamepad1.square) {
            lift.rightLift.setPower(-0.25);
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

        if (gamepad1.dpad_up) {
            runningActions.add(lift.liftTopRung());
        }


        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
    }
}
