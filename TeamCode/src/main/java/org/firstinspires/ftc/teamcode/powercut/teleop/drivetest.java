package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Intake;

import java.util.ArrayList;
import java.util.List;


@Disabled
@TeleOp
public class drivetest extends OpMode {
   private final Intake intake = new Intake();
//    private final Outtake outtake = new Outtake();
//    private final Lift lift = new Lift();
    private final Drivetrain drive = new Drivetrain();
//    private final LightSystem light = new LightSystem();

    private boolean isActionRunning = false;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    @Override
    public void init() {

        drive.init(hardwareMap);
intake.init(hardwareMap);

        drive.imu.resetYaw();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        runningActions.add(intake.transfer1Extendo());
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

        telemetry.addData("US Reads LR", "%d, %d", drive.leftUpperUS.getDistance(), drive.rightUpperUS.getDistance());

        telemetry.addData("ToF Reads LR", "%4.1f, %4.1f", drive.frontLeftToF.getDistance(DistanceUnit.MM), drive.frontRightToF.getDistance(DistanceUnit.MM));
//        if (Math.abs(-gamepad2.right_stick_y) > 0.05) {
//            isActionRunning = false;
//            double liftPower = -gamepad2.right_stick_y;
//            lift.setLiftPower(liftPower);
//        } else if (!isActionRunning) {
//            double liftPower = settings.liftHoldPower;
//            lift.setLiftPower(liftPower);
//        }
//
//        if (!isActionRunning) {
//            runningActions.clear();
//        }
//        if (gamepad1.dpad_up) {
//            runningActions.add(outtake.depositArm());
//        }
//        if (gamepad1.dpad_right) {
//            runningActions.add(outtake.specIntakeArm());
//        }
//        if (gamepad1.dpad_down) {
//            runningActions.add(outtake.transferArm());
//        }
//
//
//        if (gamepad2.dpad_up) {
//            runningActions.add(intake.intakeExtendo());
//        }
//
//        if (gamepad2.dpad_down) {
//            runningActions.add(intake.transferExtendo());
//        }
//
//
//        if (gamepad2.triangle) {
//            runningActions.clear();
//            isActionRunning = true;
//            runningActions.add(new SequentialAction(lift.liftTopRung(),new InstantAction(() -> isActionRunning = false)));
//        }
//
//        if (gamepad2.circle) {
//            runningActions.clear();
//            isActionRunning = true;
//            runningActions.add(new SequentialAction(lift.liftTopBasket(),new InstantAction(() -> isActionRunning = false)));
//        }
//
//        if (gamepad2.cross) {
//            runningActions.clear();
//            isActionRunning = true;
//            runningActions.add(new SequentialAction(lift.liftRetract(),new InstantAction(() -> isActionRunning = false)));
//        }
//
//        if (gamepad1.left_bumper) {
//            runningActions.add(outtake.closeGrip());
//        }
//
//        if (gamepad1.right_bumper) {
//            runningActions.add(outtake.openGrip());
//        }
//
//        if (gamepad1.square || gamepad2.square) {
//            runningActions.clear();
//        }
//
//
//        Intake.sampleColour sampleColour = intake.getSampleColour();
//        String colour = "";
//        if (sampleColour == Intake.sampleColour.RED) {
//            light.red();
//            colour = "Red";
//        } else if (sampleColour == Intake.sampleColour.YELLOW) {
//            light.yellow();
//            colour = "Yellow";
//        } else if (sampleColour == Intake.sampleColour.BLUE) {
//            light.blue();
//            colour = "Blue";
//        } else {
//            light.greyLarson();
//            colour = "None";
//        }
//
//        telemetry.addLine("Sample: " + colour);


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
