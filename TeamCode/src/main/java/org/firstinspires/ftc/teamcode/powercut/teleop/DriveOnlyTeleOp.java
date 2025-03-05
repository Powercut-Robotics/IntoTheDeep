package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;
import org.firstinspires.ftc.teamcode.powercut.hardware.SafeAncillary;
import org.firstinspires.ftc.teamcode.powercut.hardware.SafeAncillary.sampleColour;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class DriveOnlyTeleOp extends OpMode  {
    private final Robot robot = new Robot();
    //Hardware
    private SafeAncillary ancillary;
    private Lift lift;
    private Drivetrain drive;
    private LightSystem light;
    List<LynxModule> allHubs;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    //Localiser
    private MecanumDrive driveLoc = null;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    //System monitoring
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private Telemetry dashTelemetry = null;
    private List<Action> driveActions = new ArrayList<>();
    private List<Action> ancillaryActions = new ArrayList<>();



    private static double fullSpeedModifier = 0.75;

    private static double modifier = fullSpeedModifier;
    ElapsedTime gametimer = new ElapsedTime();





    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = robot.getDrive();
        lift = robot.getLift();
        light = robot.getLight();
        ancillary = robot.getAncillary();

        driveLoc = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        allHubs = hardwareMap.getAll(LynxModule.class);

        drive.imu.resetYaw();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashTelemetry = FtcDashboard.getInstance().getTelemetry();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        light.partyWaves();
        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
        gametimer.reset();

        telemetry.clear();

        ancillaryActions.add(new SequentialAction(
                ancillary.travelExtendo(),
                new ParallelAction(
                        ancillary.intakeTravelArm(),
                        ancillary.outtakeTransferArm(),
                        ancillary.closeGrip()
                )
        ));
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        doDrive();
        lights();
        TelemetryPacket packet = new TelemetryPacket();

        List<Action> newActions = new ArrayList<>();
        for (Action action : ancillaryActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        ancillaryActions = newActions;

        driveLoc.updatePoseEstimate();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), driveLoc.pose);
        telemetry.addData("x", driveLoc.pose.position.x);
        telemetry.addData("y", driveLoc.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(driveLoc.pose.heading.toDouble()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }


        telemetry.update();
        loopTimer.reset();
    }

    private void doDrive() {
        double x = currentGamepad1.left_stick_x;
        double y = -currentGamepad1.left_stick_y;
        double theta = currentGamepad1.right_stick_x;

        if (currentGamepad1.right_bumper) {
            modifier = 0.25;
        } else {
            modifier = fullSpeedModifier;
        }

        if (!gamepad2.cross && !gamepad2.triangle && !gamepad2.circle && !gamepad2.square && !gamepad2.dpad_left && !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_down) {
            drive.setDrivetrainPowers(x, y, theta, modifier, true);
        } else {
            drive.kill();
        }
    }



    public void lights() {
        sampleColour sampleColour = ancillary.getSampleColour();
        String colour = "None";
        if (sampleColour == sampleColour.RED) {
            colour = "Red";
            light.red();
        } else if (sampleColour == sampleColour.YELLOW) {
            colour = "Yellow";
            light.yellow();
        } else if (sampleColour == sampleColour.BLUE) {
            colour = "Blue";
            light.blue();
        } else {
            light.partyWaves();
        }

        telemetry.addData("Sample colour", colour);
    }
}



