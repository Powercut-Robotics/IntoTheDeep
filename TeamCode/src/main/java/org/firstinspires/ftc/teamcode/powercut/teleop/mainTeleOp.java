package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Intake;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Outtake;
import org.firstinspires.ftc.teamcode.powercut.settings;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp
public class mainTeleOp extends OpMode {
    private final Outtake outtake = new Outtake();
    private final Intake intake = new Intake();
    private final Lift lift = new Lift();
    private final Drivetrain drive = new Drivetrain();

    private MecanumDrive driveLoc = null;
    private final LightSystem light = new LightSystem();

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double lastX = 0;
    private double lastY = 0;
    private double lastTheta = 0;



    private final PIDEx yawLockPID = new PIDEx(settings.yawLockCoefficients);
    AngleController yawController = new AngleController(yawLockPID);
    boolean yawLock = false;
    boolean firstLock = false;
    double setYaw = 0;
    boolean isHang = false;

    private double modifier = 1;

    private final FtcDashboard dash = FtcDashboard.getInstance();
    private Telemetry dashTelemetry = null;
    private List<Action> driveActions = new ArrayList<>();
    private List<Action> ancillaryActions = new ArrayList<>();
    List<LynxModule> allHubs;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    ElapsedTime gametimer = new ElapsedTime();

    @Override
    public void init() {
        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        lift.init(hardwareMap);
        drive.init(hardwareMap);
        light.init(hardwareMap);
        driveLoc = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        allHubs = hardwareMap.getAll(LynxModule.class);

        drive.imu.resetYaw();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashTelemetry = FtcDashboard.getInstance().getTelemetry();

        light.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);

        setYaw = drive.getYaw();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void start() {
        light.greyLarson();
        loopTimer.reset();
        gametimer.reset();

        telemetry.clear();

        ancillaryActions.add(new ParallelAction(
                intake.travelArm(),
                intake.transfer2Extendo(),
                outtake.transferArm(),
                outtake.closeGrip()
                )
        );
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        TelemetryPacket packet = new TelemetryPacket();

        drive();
        ancillary();
        lights();

        List<Action> newActions = new ArrayList<>();
        for (Action action : driveActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        driveActions = newActions;

        newActions.clear();
        for (Action action : ancillaryActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        ancillaryActions = newActions;

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        driveLoc.updatePoseEstimate();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), driveLoc.pose);
        telemetry.addData("x", driveLoc.pose.position.x);
        telemetry.addData("y", driveLoc.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(driveLoc.pose.heading.toDouble()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.update();
        loopTimer.reset();
    }

    public void drive() {
        double yaw = drive.getYaw();
        double yawRad = Math.toRadians(yaw);
        double x = currentGamepad1.left_stick_x;
        double y = -currentGamepad1.left_stick_y;
        double theta = currentGamepad1.right_stick_x;

        if (Math.abs(x) > 0.05 || Math.abs(y) > 0.05 || Math.abs(theta) > 0.05) {
            drive.isDriveAction = false;
            driveActions.clear();
        }

        dashTelemetry.addData("X:", x);
        dashTelemetry.addData("Y:", y);
        dashTelemetry.addData("Theta", theta);
        dashTelemetry.addData("Yaw:", yaw);

        yawLock = (Math.abs(theta) < 0.01);
        if (!yawLock || drive.isDriveAction) {
            firstLock = true;
            setYaw = yawRad;
        } else if (!isHang) {
            if (firstLock && drive.getRadialVelocity() < 5) {
                setYaw = yawRad;
                firstLock = false;
            }

            if (!firstLock) {
                theta = yawController.calculate(setYaw, yawRad);

                if (Math.abs(theta) < 0.05) {
                    theta = 0;
                }
            }
        }

        double x_rotated = x * Math.cos(-yawRad) - y * Math.sin(-yawRad);
        double y_rotated = x * Math.sin(-yawRad) + y * Math.cos(-yawRad);

        if (!drive.isDriveAction) {
            if ((Math.abs(x_rotated-lastX) > settings.driveCacheAmount) || (Math.abs(y_rotated-lastY) > settings.driveCacheAmount) || (Math.abs(theta-lastTheta) > settings.driveCacheAmount)){
                drive.setDrivetrainPowers(x_rotated, y_rotated, theta, modifier);
                lastX = x_rotated;
                lastY = y_rotated;
                lastTheta = theta;
            }
        }
    }

    public void ancillary() {
        if (lift.isLiftAvailable && lift.liftStop.getState() && !isHang) {
          lift.setLiftPower(settings.liftHoldPower);
        }

        if (!lift.liftStop.getState() && lift.isDescending && !isHang) {
            lift.kill();
        }

        telemetry.addData("Left Lift Pos", lift.leftLift.getCurrentPosition());
        telemetry.addData("Left Lift Power", lift.leftLift.getPower());
        telemetry.addData("Right Lift Pos", lift.rightLift.getCurrentPosition());
        telemetry.addData("Right Lift Power", lift.rightLift.getPower());

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            driveActions.add(drive.alignSubmersible());
        } else if (!currentGamepad1.dpad_up && previousGamepad1.dpad_up) {
                driveActions.clear();
                drive.kill();
            }


        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            ancillaryActions.clear();
            modifier = 0.2;
            telemetry.clear();
            telemetry.addLine("Intaking...");
            ancillaryActions.add(new SequentialAction(
                            intake.lowerArm(),
                    new ParallelAction(
                            //intake.lowerArm(),
                            intake.intakeExtendo(),
                            intake.intakeAction(),
                            outtake.transferArm(),
                            outtake.openGrip(),
                            lift.liftRetract()
                    )
                    ));
        } else if (!currentGamepad2.dpad_up && previousGamepad2.dpad_up) {
            ancillaryActions.clear();
            modifier = 1;
            telemetry.clear();
            telemetry.addLine("Intake ending...");
            ancillaryActions.add(new SequentialAction(
                    intake.wheelHaltAction(),
                    intake.lowerArm(),
                    new ParallelAction(
                            intake.travelArm(),
                            intake.transfer1Extendo()
                    ),
                    intake.transferArm(),
                    intake.transfer2Extendo(),
                    intake.transferAction(),
                    outtake.closeGrip(),
                    new InstantAction(() -> modifier = 1),
                    new InstantAction(() -> telemetry.clear()),
                    new InstantAction(() -> telemetry.addLine("Intake complete"))
            ));
        }

        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            ancillaryActions.clear();
            modifier = 0.2;
            telemetry.clear();
            telemetry.addLine("Intaking spec...");
            driveActions.add(drive.alignWall());
            ancillaryActions.add(new SequentialAction(
                    new RaceAction(
                            intake.travelArm(),
                            new SleepAction(0.2)
                    ),
                        intake.clearanceExtendo(),

                        new ParallelAction(
                                lift.liftRetract(),
                                outtake.specIntakeArm()
                        ),
                        intake.transfer2Extendo(),
                        outtake.openGrip()
                    ));


        } else if (!currentGamepad2.dpad_down && previousGamepad2.dpad_down) {
            driveActions.clear();
            ancillaryActions.clear();
            modifier = 1;
            ancillaryActions.add(new SequentialAction(
                    outtake.closeGrip(),
                    outtake.travelArm(),
                    new InstantAction(() -> telemetry.clear()),
                    new InstantAction(() -> telemetry.addLine("Intake spec complete"))
            ));
        }

        if (currentGamepad2.square && !previousGamepad2.square) {
            ancillaryActions.clear();
            telemetry.clear();
            telemetry.addLine("Depositing sample...");
            driveActions.add(drive.alignBasket());
            ancillaryActions.add(
                    new SequentialAction(
                            lift.liftTopBasket(),
                            outtake.depositSampArm()
                    )
            );
        } else if (!currentGamepad2.square && previousGamepad2.square) {
            ancillaryActions.clear();
            telemetry.clear();
            telemetry.addLine("Depositing sample end...");
            driveActions.clear();
            drive.kill();
            ancillaryActions.add(
                    new SequentialAction(
                            outtake.openGrip(),
                            outtake.transferArm(),
                            new ParallelAction(
                                    outtake.closeGrip(),
                                    lift.liftRetract()
                            ),
                            new InstantAction(() -> telemetry.clear()),
                            new InstantAction(() -> telemetry.addLine("Depositing sample complete"))
                    )
            );
        }

        if (currentGamepad2.circle && !previousGamepad2.circle) {
            ancillaryActions.clear();
            telemetry.clear();
            telemetry.addLine("Depositing spec...");
            driveActions.add(drive.alignRung());
            ancillaryActions.add(new SequentialAction(
                            lift.liftTopRung(),
                            outtake.depositSpecArm()
                    )
            );
        } else if (!currentGamepad2.circle && previousGamepad2.circle) {
            ancillaryActions.clear();
            telemetry.clear();
            driveActions.clear();
            drive.kill();
            ancillaryActions.add(new SequentialAction(
                            outtake.relaxGrip(),
                            lift.liftTopRungAttached(),
                            outtake.openGrip(),
                            new ParallelAction(
                                    outtake.transferArm(),
                                    new SequentialAction(
                                            new SleepAction(0.5),
                                            lift.liftRetract()
                                    ),
                                    outtake.closeGrip()

                            ),
                            new InstantAction(() -> telemetry.clear()),
                            new InstantAction(() -> telemetry.addLine("Depositing spec complete"))
                    )
            );
        }

        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            ancillaryActions.clear();
            lift.kill();
            ancillaryActions.add(
                    lift.liftRetractSensor()
            );
            isHang = false;
        }


        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            driveActions.clear();
            ancillaryActions.clear();
            isHang = true;
            ancillaryActions.add(lift.liftPreHang());
            telemetry.clear();
        } else if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
            telemetry.addLine("Hanging...");
            driveActions.clear();
            ancillaryActions.clear();
            ancillaryActions.add(lift.liftHang());
        }

        if ((currentGamepad1.dpad_left && !previousGamepad1.dpad_left) || (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)) {
            ancillaryActions.clear();
            driveActions.clear();
            lift.kill();
            drive.kill();
            ancillaryActions.add(new ParallelAction(
                    intake.relaxSystem(),
                    outtake.relaxSystem()
            ));
            isHang = false;
        }

        if (currentGamepad1.triangle || currentGamepad2.triangle) {
            ancillaryActions.clear();
            driveActions.clear();
            lift.kill();
            isHang = false;
            ancillaryActions.add(
                    new SequentialAction(
                            new ParallelAction(
                                    intake.lowerArmSafe(),
                                    intake.wheelHaltAction(),
                                    intake.transfer1Extendo(),
                                    outtake.transferArm(),
                                    outtake.closeGrip()
                            ),
                            new ParallelAction(
                                    lift.liftRetract()
                            )
                    )
            );
        }
    }

    public void lights() {
        Intake.sampleColour sampleColour = intake.getSampleColour();
        String colour = "None";
        if (sampleColour == Intake.sampleColour.RED) {
            colour = "Red";
            light.red();
        } else if (sampleColour == Intake.sampleColour.YELLOW) {
            colour = "Yellow";
            light.yellow();
        } else if (sampleColour == Intake.sampleColour.BLUE) {
            colour = "Blue";
            light.blue();
        } else if (gametimer.seconds() > 120 && gametimer.seconds() < 122) {
            light.redStrobe();
        } else if (gametimer.seconds() > 122) {
            light.redLarson();
        } else {
            light.greyLarson();
        }

        telemetry.addData("Sample colour", colour);
    }

}
