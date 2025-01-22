package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Intake;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Outtake;
import org.firstinspires.ftc.teamcode.powercut.settings;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class testing extends OpMode {
    private final Intake intake = new Intake();
    private final Outtake outtake = new Outtake();
    private final Lift lift = new Lift();
    private final Drivetrain drive = new Drivetrain();
    private final LightSystem light = new LightSystem();

    private boolean isActionRunning = false;

    private Gamepad gamepad1last;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    @Override
    public void init() {
        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        lift.init(hardwareMap);
        drive.init(hardwareMap);
        light.init(hardwareMap);

        drive.imu.resetYaw();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        light.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);

        gamepad1last = gamepad1;
        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        double yaw = drive.getYaw();
        double yawRad = Math.toRadians(yaw);
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double theta = gamepad1.right_stick_x;
        double x_rotated = x * Math.cos(-yawRad) - y * Math.sin(-yawRad);
        double y_rotated = x * Math.sin(-yawRad) + y * Math.cos(-yawRad);
        drive.setDrivetrainPowers(x_rotated, y_rotated, theta,1);

        telemetry.addData("X:", x);
        telemetry.addData("Y:", y);
        telemetry.addData("XRot:", x_rotated);
        telemetry.addData("YRot:", y_rotated);
        telemetry.addData("Theta", theta);
        telemetry.addData("Yaw:", yaw);
        telemetry.addData("Lift Pos", "%d, %d", lift.leftLift.getCurrentPosition(), lift.rightLift.getCurrentPosition());
        telemetry.addData("US Reads LR", "%d, %d", drive.leftUpperUS.getDistance(), drive.rightUpperUS.getDistance());

        telemetry.addData("Lift Power", "%4.3f, %4.3f", lift.leftLift.getPower(), lift.rightLift.getPower());
        telemetry.addData("Colour Sensor Values (RGBA), Range", "%d, %d, %d, %d", intake.colourSensor.red(), intake.colourSensor.green(), intake.colourSensor.blue(), intake.colourSensor.alpha());
        telemetry.addData("ToF Reads LR", "%4.1f, %4.1f", drive.frontLeftToF.getDistance(DistanceUnit.MM), drive.frontRightToF.getDistance(DistanceUnit.MM));

        telemetry.addData("Action running", isActionRunning);
        telemetry.addData("gamepad 2 y", -gamepad2.right_stick_y);
        if (Math.abs(-gamepad2.right_stick_y) > 0.05) {
            isActionRunning = false;
            double liftPower = -gamepad2.right_stick_y;
            lift.setLiftPower(liftPower);
        } else if (!isActionRunning) {
            double liftPower = settings.liftHoldPower;
            lift.setLiftPower(liftPower);
        }

//        if (!isActionRunning) {
//            runningActions.clear();
//        }
        if (gamepad1.triangle && ! gamepad1last.triangle) {
            runningActions.add(new SequentialAction(
                    new ParallelAction(
                            intake.intakeExtendo(),
                            intake.lowerArm(),
                            intake.intakeAction(),
                            outtake.transferArm(),
                            outtake.openGrip()
                    )

            ));
        } else if (gamepad1last.triangle && !gamepad1last.triangle) {
            runningActions.add(new SequentialAction(
                    new ParallelAction(
                            intake.travelArm(),
                            new SequentialAction(new SleepAction(0.7), intake.transferExtendo())
                    ),
                    intake.transferArm(),
                    intake.transferAction(),
                    outtake.closeGrip()
            ));
        }

        if (gamepad1.circle && !gamepad1last.circle) {
            runningActions.add(new SequentialAction(

                    lift.liftTopBasket(),
                    outtake.depositArm(),
                    outtake.openGrip()
            ));
        } else if (!gamepad1.circle && gamepad1last.circle) {
            runningActions.add(new SequentialAction(
                    outtake.openGrip(),
                    new ParallelAction(
                            outtake.transferArm(),
                            outtake.openGrip(),
                            lift.liftRetract()
                    )
            ));
        }


        if (gamepad1.dpad_up) {
            runningActions.add(intake.intakeExtendo());
        }

        if (gamepad1.dpad_down) {
            runningActions.add(intake.transferExtendo());
        }


        if (gamepad2.circle) {
            runningActions.clear();
            isActionRunning = true;
            runningActions.add(new SequentialAction(lift.liftTopRung(),new InstantAction(() -> isActionRunning = false)));
        }

        if (gamepad2.triangle) {
            runningActions.clear();
            isActionRunning = true;
            runningActions.add(new SequentialAction(lift.liftTopBasket(),new InstantAction(() -> isActionRunning = false)));
        }

        if (gamepad2.cross) {
            runningActions.clear();
            isActionRunning = true;
            runningActions.add(new SequentialAction(lift.liftRetract(),new InstantAction(() -> isActionRunning = false)));
        }



        if (gamepad1.square || gamepad2.square) {
            runningActions.clear();
        }


        Intake.sampleColour sampleColour = intake.getSampleColour();
        String colour = "";
        if (sampleColour == Intake.sampleColour.RED) {
            light.red();
            colour = "Red";
        } else if (sampleColour == Intake.sampleColour.YELLOW) {
            light.yellow();
            colour = "Yellow";
        } else if (sampleColour == Intake.sampleColour.BLUE) {
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

        gamepad1last = gamepad1;
        telemetry.addData("running acgtions", runningActions.size());
        telemetry.addData("new acgtions", newActions.size());
        telemetry.update();
    }
}
