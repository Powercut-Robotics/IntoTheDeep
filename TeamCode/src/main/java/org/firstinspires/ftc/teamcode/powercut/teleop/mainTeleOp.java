package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Intake;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Outtake;
import org.firstinspires.ftc.teamcode.powercut.settings;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class mainTeleOp extends OpMode {
    private final Outtake outtake = new Outtake();
    private final Intake intake = new Intake();
    private final Lift lift = new Lift();
    private final Drivetrain drive = new Drivetrain();
    private final LightSystem light = new LightSystem();

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double lastX = 0;
    private double lastY = 0;
    private double lastTheta = 0;

    private boolean authorised = false;
    private enum sequence {
        TopBasket,
        BottomBasket,
        TopRung,
        BottomRung,
        Intake
    }
    private sequence current = null;
    private enum basket {
        LiftExtend,
        Release
    }
    private basket basketCurrent = null;
    private enum rung {
        LiftExtend,
        LowerLift,

        Release
    }
    private rung rungCurrent = null;
    private enum intakeEnum {
        GripOpen,
        ArmDown,
        GripClosed,
        ArmUp
    }
    private intakeEnum intakeCurrent = null;

    private boolean authLast = false;

    // Actions
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private Gamepad gamepad2Last;

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        outtake.init(hardwareMap);
        lift.init(hardwareMap);
        drive.init(hardwareMap);
        light.init(hardwareMap);

        drive.imu.resetYaw();

        light.confetti();
        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void start() {
        light.greyLarson();
        loopTimer.reset();
        gamepad2Last = gamepad2;
    }

    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();
        double yaw = drive.getYaw();
        telemetry.addData("Yaw", yaw);
        double yawRad = Math.toRadians(yaw);
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double theta = gamepad1.right_stick_x;
        double x_rotated = y * Math.cos(yawRad) - x * Math.sin(yawRad);
        double y_rotated = y * Math.sin(yawRad) + x * Math.cos(yawRad);

        //cache motor control for faster loop times
        if ((Math.abs(x_rotated-lastX) > settings.driveCacheAmount) || (Math.abs(y_rotated-lastY) > settings.driveCacheAmount) || (Math.abs(theta-lastTheta) > settings.driveCacheAmount)){
            drive.setDrivetrainPowers(x_rotated, y_rotated, theta, 1);
            lastX = x_rotated;
            lastY = y_rotated;
            lastTheta = theta;
        }

        if (current == sequence.Intake) {
            gripBasedLightControl();
        }

        ancillarySystemControl();

        String routine = "";
        if (current == sequence.Intake) {
            routine = "Intake";
        } else if (current == sequence.TopBasket) {
            routine = "Top Basket";
        } else if (current == sequence.BottomBasket) {
            routine = "Bottom Basket";
        } else if (current == sequence.TopRung) {
            routine = "Top Rung";
        } else if (current == sequence.BottomRung) {
            routine = "Bottom Rung";
        }

        Intake.sampleColour sampleColour = intake.getSampleColour();
        String colour = "";
        if (sampleColour == Intake.sampleColour.RED) {
            light.red();
            colour = "Red";
        } else if (sampleColour == Intake.sampleColour.YELLOW) {
            light.yellow();
            colour = "Green";
        } else if (sampleColour == Intake.sampleColour.BLUE) {
            light.blue();
            colour = "Blue";
        } else {
            light.greyLarson();
            colour = "None";
        }

        telemetry.addLine("Sample: " + colour);
        telemetry.addLine("Routine: " + routine);
        telemetry.addData("Loop Timer", loopTimer.time(TimeUnit.MILLISECONDS));
        telemetry.addData("Lift Pos", "%d, %d", lift.leftLift.getCurrentPosition(), lift.rightLift.getCurrentPosition());

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        gamepad2Last = gamepad2;
        telemetry.update();
        loopTimer.reset();
    }

    private void gripBasedLightControl() {
        Intake.sampleColour sampleColour = intake.getSampleColour();

        if (sampleColour == Intake.sampleColour.RED) {
            light.red();
        } else if (sampleColour == Intake.sampleColour.YELLOW) {
            light.yellow();
        } else if (sampleColour == Intake.sampleColour.BLUE) {
            light.blue();
        } else {
            light.greyLarson();
        }
    }

    private void ancillarySystemControl() {
        if (lift.isLiftAvailable) {
            lift.setLiftPower(settings.liftHoldPower);
        }
//Intake
        if (gamepad2.dpad_up && !gamepad2Last.dpad_up) {
            current = sequence.Intake;
            runningActions.add(
                            new ParallelAction(
                                    intake.intakeExtendo(),
                                    intake.intake(),
                                    lift.liftRetract(),
                                    outtake.transferArm(),
                                    outtake.openGrip(),
                                    intake.lowerArm()
                            )
            );
        } else if (!gamepad2.dpad_up && gamepad2Last.dpad_up && (current == sequence.Intake)) {
            current = null;
            runningActions.add(
                    new SequentialAction(
                            intake.transfer(),
                            outtake.closeGrip()
                    )
            );

        }

// Top Basket
        if (gamepad2.dpad_down && !gamepad2Last.dpad_down) {
            current = sequence.TopBasket;
            runningActions.add(
                   new SequentialAction(
                           new ParallelAction(
                           lift.liftTopRung(),
                           drive.alignBasket()
                           ),
                           outtake.depositArm()
                   )
            );
        } else if (!gamepad2.dpad_down && gamepad2Last.dpad_down && (current == sequence.TopBasket)) {
            current = null;
            runningActions.add(
                    new SequentialAction(
                            outtake.openGrip(),
                            new ParallelAction(
                                    lift.liftRetract(),
                                    outtake.transferArm()
                            )

                    )
            );

        }

        //top rung
        if (gamepad2.dpad_left && !gamepad2Last.dpad_left) {
            current = sequence.TopRung;
            runningActions.add(
                    new ParallelAction(
                            outtake.depositArm(),
                            lift.liftTopRung()
                    )
            );
        } else if (!gamepad2.dpad_left && gamepad2Last.dpad_left && (current == sequence.TopRung)) {
            current = null;
            runningActions.add(
                    new SequentialAction(
                            outtake.specIntakeArm(),
                            new ParallelAction(
                                    outtake.openGrip(),
                                    lift.liftRetract()
                            ),
                            outtake.transferArm()
                    )
            );
        }


    }

}
