package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.powercut.hardware.Ancillary;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class DriveTeleOp extends OpMode  {
    private final Robot robot = new Robot();
    //Hardware
    private Ancillary ancillary;
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

    public enum samplePosition {
        INTAKE,
        TRAY,
        OUTTAKE,
        NONE
    }

    public class sampleStatus {
        Ancillary.sampleColour colour;
        samplePosition position;
    }

    public sampleStatus status = new sampleStatus();

    //Defaults

    private final Action homeAncillary = new SequentialAction(
            new InstantAction(() -> telemetry.clear()),
            new InstantAction(() -> telemetry.addLine("Homing")),
            ancillary.clearanceExtendo(),
            new ParallelAction(
                ancillary.intakeTravelArm(),
                ancillary.outtakeTransferArm(),
                ancillary.closeGrip()
            )
    );

    private final Action transferAction = new SequentialAction(
            new ParallelAction(
                    lift.liftRetract(),
                    ancillary.outtakeTransferArm(),
                    ancillary.openGrip()
            ),
            new ParallelAction(
                    ancillary.transferExtendo(),
                    ancillary.intakeTransferArm()
            ),
            ancillary.transferAction(),
            ancillary.closeGrip(),
            new InstantAction(() ->
                status.position = samplePosition.TRAY
            )
    );


    private final Action transferDisengage = new SequentialAction(
            ancillary.clearanceExtendo(),
            ancillary.intakeTravelArm()
    );

    private boolean extendoControl = false;

    //Game monitoring
    boolean isHang = false;
    boolean completedAction = false;
    private double modifier = 1;
    ElapsedTime gametimer = new ElapsedTime();





    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = robot.getDrive();
        lift = robot.getLift();
        light = robot.getLight();
        ancillary = robot.getAncillary();

        status.colour = Ancillary.sampleColour.NONE;
        status.position = samplePosition.NONE;

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
        light.greyLarson();
        loopTimer.reset();
        gametimer.reset();

        telemetry.clear();

        ancillaryActions.add(homeAncillary);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        doDrive();
        doAncillary();
        lights();

        TelemetryPacket packet = new TelemetryPacket();


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

        telemetry.addData("Intake active", ancillary.intakeActive);
        telemetry.update();
        loopTimer.reset();
    }

    private void doDrive() {
        double x = currentGamepad1.left_stick_x;
        double y = -currentGamepad1.left_stick_y;
        double theta = currentGamepad1.right_stick_x;

        if (currentGamepad1.cross && !previousGamepad1.cross) {
            telemetry.addData("Drive", "Aligning basket");
            driveActions.clear();
            driveActions.add(drive.alignBasket());
        } else if (!currentGamepad1.cross && previousGamepad1.cross) {
            telemetry.addData("Drive", "Disengaging basket");
            driveActions.clear();
            drive.kill();
            driveActions.add(drive.disengageBasket());
        }

        if (currentGamepad1.triangle && !previousGamepad1.triangle) {
            telemetry.addData("Drive", "Aligning rung");
            driveActions.clear();
            driveActions.add(drive.alignRung());
        } else if (!currentGamepad1.triangle && previousGamepad1.triangle) {
            telemetry.addData("Drive", "Aligning stopped");
            driveActions.clear();
            drive.kill();
        }

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            telemetry.addData("Drive", "Aligning wall");
            modifier = 0.25;
            driveActions.clear();
            driveActions.add(drive.alignWall());
        } else if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down) {
            telemetry.addData("Drive", "Aligning stopped");
            modifier = 1;
            driveActions.clear();
        }

        if (currentGamepad1.right_bumper) {
            modifier = 0.25;
        } else {
            modifier = 1;
        }

        drive.setDrivetrainPowers(x, y, theta, modifier,true);
    }

    public void doAncillary() {
        telemetry.addData("Position", status.position.toString());
        if (lift.isLiftAvailable && lift.liftStop.getState() && !isHang) {
            lift.setLiftPower(0.05);
        }

        if (extendoControl) {
            if (currentGamepad2.triangle) {
                telemetry.addData("Extendo", "Full");
                ancillaryActions.add(ancillary.intakeExtendo());
            } else if (currentGamepad2.circle) {
                telemetry.addData("Extendo", "Half");
                ancillaryActions.add(ancillary.halfExtendo());
            } else if (currentGamepad2.cross) {
                telemetry.addData("Extendo", "None");
                ancillaryActions.add(ancillary.clearanceExtendo());
            }
        } else {
            if (currentGamepad2.cross && !previousGamepad2.cross) {
                telemetry.addData("Main", "Top Basket");
                ancillaryActions.clear();
                if (status.position == samplePosition.TRAY) {
                    ancillaryActions.add(
                            new SequentialAction(
                                    lift.liftTopBasket(),
                                    ancillary.depositSampArm()
                            )
                    );
                } else {
                    ancillaryActions.add(
                            new SequentialAction(
                                    transferAction,
                                    transferDisengage,
                                    lift.liftTopBasket(),
                                    ancillary.depositSampArm()
                            )
                    );
                }
            } else if (!currentGamepad2.cross && previousGamepad2.cross) {
                telemetry.addData("Sample Status", "None");
                telemetry.addData("Main", "Top Basket End");
                status.position = samplePosition.NONE;
                status.colour = Ancillary.sampleColour.NONE;
                ancillaryActions.clear();
                ancillaryActions.add(
                        new SequentialAction(
                                ancillary.openGrip(),
                                ancillary.outtakeTransferArm(),
                                new SleepAction(4),
                                new ParallelAction(
                                        ancillary.closeGrip(),
                                        lift.liftRetract()
                                )
                        )
                );
            }

            if (currentGamepad2.triangle && !previousGamepad2.triangle) {
                telemetry.addData("Main", "Top Rung");
                ancillaryActions.clear();
                    ancillaryActions.add(new SequentialAction(
                                    lift.liftTopRung(),
                                    ancillary.depositSpecArm()
                            )
                    );
            } else if (!currentGamepad2.triangle && previousGamepad2.triangle) {
                telemetry.addData("Main", "Top Rung End");
                telemetry.addData("Sample Status", "None");
                status.position = samplePosition.NONE;
                status.colour = Ancillary.sampleColour.NONE;
                ancillaryActions.clear();
                ancillaryActions.add(new SequentialAction(
                                ancillary.relaxGrip(),
                                lift.liftTopRungAttached(),
                                ancillary.openGrip(),
                                new ParallelAction(
                                        ancillary.outtakeTransferArm(),
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                lift.liftRetract()
                                        ),
                                        ancillary.closeGrip()

                                )
                        )
                );
            }
        }

        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            telemetry.addData("Sample Status", "Transfering");
            ancillaryActions.add(new SequentialAction(
                    transferAction,
                    transferDisengage
            ));
        }

        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            telemetry.addData("Main", "Spec Intake");
            completedAction = false;
            ancillaryActions.clear();
            ancillaryActions.add(new SequentialAction(
                    new ParallelAction(
                            lift.liftRetract(),
                            ancillary.clearanceExtendo(),
                            ancillary.intakeTravelArm()
                    ),
                    new ParallelAction(
                            ancillary.specIntakeArm(),
                            ancillary.openGrip(),
                            new InstantAction(() -> completedAction = true)
                    )
            ));
        } else if (!currentGamepad2.dpad_down && previousGamepad2.dpad_down) {

            if (completedAction) {
                telemetry.addData("Sample Status", "Outtake");
                telemetry.addData("Main", "Spec Travel");
                completedAction = false;
                status.position = samplePosition.OUTTAKE;
                ancillaryActions.add(
                        new SequentialAction(
                                ancillary.closeGrip(),
                                new ParallelAction(
                                    ancillary.outtakeTravelArm()
                                )
                        )
                );
            } else {
                telemetry.addData("Main", "Home");
                ancillaryActions.add(
                        homeAncillary
                );
            }
        }

        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            telemetry.addData("Sample Status", "Intake");
            telemetry.addData("Main", "Extendo");
            telemetry.addData("Extendo", "Full");
            ancillaryActions.clear();
            driveActions.clear();
            ancillaryActions.add(new SequentialAction(
                    new ParallelAction(
                        ancillary.intakeExtendo(),
                        ancillary.intakeExpelArm(),
                            ancillary.spinOutAction()
                    ),
                    ancillary.expelAction()
                    )
            );
        } else if (!currentGamepad2.left_bumper && previousGamepad2.left_bumper) {
            telemetry.addData("Sample Status", "None");
            telemetry.addData("Main", "Home");
            telemetry.addData("Extendo", "Home");
            ancillaryActions.clear();
            status.position = samplePosition.NONE;
            status.colour = Ancillary.sampleColour.NONE;
            ancillaryActions.add(new ParallelAction(
                    ancillary.wheelHaltAction(),
                    ancillary.clearanceExtendo(),
                    ancillary.intakeTravelArm(),
                    ancillary.wheelHaltAction()
                    )
            );
        }

        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            telemetry.addData("Sample Status", "None");
            telemetry.addData("Main", "Intake");
            ancillaryActions.clear();
            driveActions.clear();
            modifier = 0.25;
            extendoControl = true;
            ancillaryActions.add(new SequentialAction(
                    new ParallelAction(
                            ancillary.spinUpAction(),
                            ancillary.intakeLowerArm(),
                            lift.liftRetract(),
                            ancillary.outtakeTransferArm(),
                            ancillary.openGrip()
                    )
            ));
        } else if (previousGamepad2.dpad_up && !currentGamepad2.dpad_up) {
            telemetry.addData("Sample Status", "Intake");
            telemetry.addData("Main", "Home");
            extendoControl = false;
            status.position = samplePosition.INTAKE;
            ancillaryActions.clear();
            modifier = 1;
            ancillaryActions.add(
                    new ParallelAction(
                        ancillary.wheelHaltAction(),
                            ancillary.intakeTravelArm(),
                            ancillary.clearanceExtendo()
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
            ancillary.relaxSystem();
            isHang = false;
        }

        if (currentGamepad1.square || currentGamepad2.square) {
            telemetry.addData("Extendo", "Home");
            telemetry.addData("Main", "Home");
            ancillaryActions.clear();
            driveActions.clear();
            lift.kill();
            drive.kill();
            isHang = false;
            ancillaryActions.add(
                    new SequentialAction(
                            homeAncillary,
                            new ParallelAction(
                                    lift.liftRetract()
                            )
                    )
            );
        }

    }

    public void lights() {
        Ancillary.sampleColour sampleColour = ancillary.getSampleColour();
        String colour = "None";
        if (sampleColour == Ancillary.sampleColour.RED) {
            status.colour = Ancillary.sampleColour.RED;
            colour = "Red";
            light.red();
        } else if (sampleColour == Ancillary.sampleColour.YELLOW) {
            status.colour = Ancillary.sampleColour.YELLOW;
            colour = "Yellow";
            light.yellow();
        } else if (sampleColour == Ancillary.sampleColour.BLUE) {
            status.colour = Ancillary.sampleColour.BLUE;
            colour = "Blue";
            light.blue();
        } else if (gametimer.seconds() > 120 && gametimer.seconds() < 122) {
            light.redStrobe();
        } else if (gametimer.seconds() > 122) {
            light.redLarson();
        } else {
            light.greyLarson();
        }

        if (gametimer.seconds() > 120 && gametimer.seconds() < 120.2) {
            gamepad1.setLedColor(1.0,0.0,0.0, 500);
            gamepad2.setLedColor(1.0,0.0,0.0, 500);
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        telemetry.addData("Sample colour", colour);
    }
}



