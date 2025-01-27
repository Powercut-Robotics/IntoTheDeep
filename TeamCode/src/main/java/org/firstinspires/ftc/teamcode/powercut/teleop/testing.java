package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    List<LynxModule> allHubs;
    private ElapsedTime loopTime = new ElapsedTime();
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();


    private double lastX = 0;
    private double lastY = 0;
    private double lastTheta = 0;

    private PIDEx yawLockPID = new PIDEx(settings.yawLockCoefficients);
    AngleController yawController = new AngleController(yawLockPID);
    boolean yawLock = false;
    boolean firstLock = false;
    double setYaw = 0;
    
    @Override
    public void init() {
        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        lift.init(hardwareMap);
        drive.init(hardwareMap);
        light.init(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);

        drive.imu.resetYaw();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        light.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);

        setYaw = drive.getYaw();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void loop() {
        loopTime.reset();
        TelemetryPacket packet = new TelemetryPacket();
        drive();

        telemetry.addData("Lift Pos", "%d, %d", lift.leftLift.getCurrentPosition(), lift.rightLift.getCurrentPosition());
        telemetry.addData("US Reads LR", "%d, %d", drive.leftUpperUS.getDistance(), drive.rightUpperUS.getDistance());

        telemetry.addData("Lift Power", "%4.3f, %4.3f", lift.leftLift.getPower(), lift.rightLift.getPower());
        telemetry.addData("Colour Sensor Values (RGBA), Range", "%d, %d, %d, %d", intake.colourSensor.red(), intake.colourSensor.green(), intake.colourSensor.blue(), intake.colourSensor.alpha());
        telemetry.addData("ToF Reads LR", "%4.1f, %4.1f", drive.frontLeftToF.getDistance(DistanceUnit.MM), drive.frontRightToF.getDistance(DistanceUnit.MM));

        double leftLowerMVout = drive.leftLowerUS.getVoltage() * 1000;
        double rightLowerMVout = drive.rightLowerUS.getVoltage() * 1000;

        telemetry.addData("Analog", "%5.2f, %5.2f", (leftLowerMVout*520)/3300, (rightLowerMVout*520)/3300);
        telemetry.addData("gamepad 2 y", -gamepad2.right_stick_y);
        telemetry.addData("Yaw from US", drive.getYawFromUS());
        telemetry.addData("Lift Stop", !lift.liftStop.getState());

        if (Math.abs(-gamepad2.right_stick_y) > 0.05) {
            isActionRunning = false;
            double liftPower = -gamepad2.right_stick_y;
            lift.setLiftPower(liftPower);
        } else if (!isActionRunning && lift.liftStop.getState()) {
            double liftPower = settings.liftHoldPower;
            lift.setLiftPower(liftPower);
        } else if (!lift.liftStop.getState()) {
            lift.kill();
        }

//        if (!isActionRunning) {
//            runningActions.clear();
//        }
        
        if (gamepad1.triangle) {
            runningActions.clear();
            isActionRunning = true;
            runningActions.add(new SequentialAction(
                    new ParallelAction(
                            intake.intakeExtendo(),
                            intake.lowerArm(),
                            intake.intakeAction(),
                            outtake.transferArm(),
                            outtake.openGrip()
                    ),
                    new ParallelAction(
                            intake.travelArm(),
                            new SequentialAction(new SleepAction(0.7), intake.transferExtendo())
                    ),
                    intake.transferArm(),
                    intake.transferAction(),
                    outtake.closeGrip(),
            new InstantAction(() -> isActionRunning = false)
            ));
        }

        if (gamepad1.circle) {
            runningActions.clear();
            isActionRunning = true;
            runningActions.add(new SequentialAction(
                    lift.liftTopBasket(),
                    new InstantAction(() -> isActionRunning = false),
                    outtake.depositArm(),
                    outtake.openGrip(),
                    new ParallelAction(
                            outtake.closeGrip(),
                            outtake.transferArm(),
                            new InstantAction(() -> isActionRunning = true),
                            lift.liftRetract()
                    ),
                    new InstantAction(() -> isActionRunning = false)
            ));
        }

        if (gamepad1.cross) {
            runningActions.clear();
            isActionRunning = true;
            runningActions.add(new SequentialAction(
                   lift.liftTopRung(),
                    new InstantAction(() -> isActionRunning = false),
                    outtake.depositArm(),
                    new InstantAction(() -> isActionRunning = true),
                    lift.liftTopRungAttached(),
                    outtake.openGrip(),
                    new ParallelAction(
                            lift.liftRetract(),
                            new SequentialAction(
                                    new SleepAction(0.3),
                                    new ParallelAction(
                                            outtake.closeGrip(),
                                            outtake.transferArm()
                                    )
                            )
                    ),
                    new InstantAction(() -> isActionRunning = false)
            ));
        }


        if (gamepad1.dpad_up) {
            drive.isDriveAction = true;
            runningActions.add(new SequentialAction(drive.alignRung(), new InstantAction(() -> drive.isDriveAction = false)));
        }

        if (gamepad1.dpad_down) {
            drive.isDriveAction = true;
            runningActions.add(new SequentialAction(drive.alignBasket(), new InstantAction(() -> drive.isDriveAction = false)));
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
            runningActions.add(new SequentialAction(lift.liftHang()));
        }



        if (gamepad1.square || gamepad2.square) {
            isActionRunning = false;
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
            light.confetti();
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

        telemetry.addData("running actions", runningActions.size());

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        telemetry.addData("radial velocity", drive.getRadialVelocity());

        telemetry.addData("Loop Time", loopTime);
        telemetry.update();
    }

    public void drive() {
        double yaw = drive.getYaw();
        double yawRad = Math.toRadians(yaw);
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double theta = gamepad1.right_stick_x;

        if (Math.abs(x) > 0.05 || Math.abs(y) > 0.05 || Math.abs(theta) > 0.05) {
            drive.isDriveAction = false;
        }

        telemetry.addData("X:", x);
        telemetry.addData("Y:", y);
        telemetry.addData("Theta", theta);
        telemetry.addData("Yaw:", yaw);

        yawLock = (Math.abs(theta) < 0.01);
        if (!yawLock) {
            firstLock = true;
            setYaw = yawRad;
        } else {
            if (firstLock && drive.getRadialVelocity() < 5) {
                setYaw = yawRad;
                firstLock = false;
            }
            
            if (!firstLock) {
                theta = yawController.calculate(setYaw, yawRad);

                if (theta < 0.05) {
                    theta = 0;
                }
            }
        }

        double x_rotated = x * Math.cos(-yawRad) - y * Math.sin(-yawRad);
        double y_rotated = x * Math.sin(-yawRad) + y * Math.cos(-yawRad);

        if (!drive.isDriveAction) {
            if ((Math.abs(x_rotated-lastX) > settings.driveCacheAmount) || (Math.abs(y_rotated-lastY) > settings.driveCacheAmount) || (Math.abs(theta-lastTheta) > settings.driveCacheAmount)){
                drive.setDrivetrainPowers(x_rotated, y_rotated, theta, 1);
                lastX = x_rotated;
                lastY = y_rotated;
                lastTheta = theta;
            }
        }
    }
}
