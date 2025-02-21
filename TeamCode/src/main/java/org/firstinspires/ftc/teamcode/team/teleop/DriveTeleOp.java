package org.firstinspires.ftc.teamcode.team.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.team.hardware.Ancillary;
import org.firstinspires.ftc.teamcode.team.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.team.hardware.Lift;
import org.firstinspires.ftc.teamcode.team.hardware.LightSystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class DriveTeleOp extends OpMode  {

    //Hardware
    private final Ancillary ancillary = new Ancillary();
    private final Lift lift = new Lift();
    private final Drivetrain drive = new Drivetrain();
    private final LightSystem light = new LightSystem();
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

    public sampleStatus status;

    //Defaults

    private Action homeAncillary = new ParallelAction(
            ancillary.intakeTravelArm(),
            ancillary.outtakeTransferArm(),
            ancillary.travelExtendo(),
            ancillary.closeGrip()
    );

    private Action transferAction = new SequentialAction(
            new ParallelAction(
                    ancillary.intakeTransferArm(),
                    ancillary.transferExtendo(),
                    lift.liftRetract(),
                    ancillary.outtakeTransferArm(),
                    ancillary.openGrip()
            ),
            ancillary.transferAction(),
            ancillary.closeGrip(),
            new InstantAction(() ->
                status.position = samplePosition.TRAY
            )
    );

    private boolean extendoControl = false;

    //Game monitoring
    boolean isHang = false;
    boolean completedAction = false;
    private double modifier = 1;
    ElapsedTime gametimer = new ElapsedTime();

    private enum AncillaryActions {
        INTAKE_SAMP,
        INTAKE_SPEC,
        OUTTAKE_BASKET,
        OUTTAKE_RUNG,
        TRANSFER,
        NONE
    }

    AncillaryActions runningAction = AncillaryActions.NONE;



    @Override
    public void init() {
        ancillary.init(hardwareMap);
        lift.init(hardwareMap);
        drive.init(hardwareMap);
        light.init(hardwareMap);
        //driveLoc = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        allHubs = hardwareMap.getAll(LynxModule.class);

        drive.imu.resetYaw();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashTelemetry = FtcDashboard.getInstance().getTelemetry();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


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

        ancillaryActions.add(new ParallelAction(
                        ancillary.intakeTravelArm(),
                        ancillary.travelExtendo(),
                        ancillary.intakeTransferArm(),
                        ancillary.closeGrip()
                )
        );
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



//        driveLoc.updatePoseEstimate();
//        packet.fieldOverlay().setStroke("#3F51B5");
//        Drawing.drawRobot(packet.fieldOverlay(), driveLoc.pose);
//        telemetry.addData("x", driveLoc.pose.position.x);
//        telemetry.addData("y", driveLoc.pose.position.y);
//        telemetry.addData("heading (deg)", Math.toDegrees(driveLoc.pose.heading.toDouble()));
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);

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

        drive.setDrivetrainPowers(x, y, theta, modifier,true);
    }

    public void doAncillary() {
        if (lift.isLiftAvailable && lift.liftStop.getState() && !isHang) {
            lift.setLiftPower(0.05);
        }

        if (extendoControl) {
            if (currentGamepad2.triangle) {
                ancillaryActions.add(ancillary.intakeExtendo());
            } else if (currentGamepad2.circle) {
                ancillaryActions.add(ancillary.halfExtendo());
            } else if (currentGamepad2.cross) {
                ancillaryActions.add(ancillary.travelExtendo());
            }
        } else {
            if (currentGamepad2.cross && !previousGamepad2.cross) {
                ancillaryActions.clear();
                driveActions.add(drive.alignBasket());
                ancillaryActions.add(
                        new SequentialAction(
                                lift.liftTopBasket(),
                                ancillary.depositSampArm()
                        )
                );
            } else if (!currentGamepad2.cross && previousGamepad2.cross) {
                status.position = samplePosition.NONE;
                status.colour = Ancillary.sampleColour.NONE;
                ancillaryActions.clear();
                driveActions.clear();
                drive.kill();
                driveActions.add(drive.disengageBasket());
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
                ancillaryActions.clear();
                if (status.position == samplePosition.INTAKE) {
                    ancillaryActions.add(new SequentialAction(
                                    transferAction,
                                    lift.liftTopRung(),
                                    ancillary.depositSpecArm()
                            )
                    );
                } else {
                    ancillaryActions.add(new SequentialAction(
                                    lift.liftTopRung(),
                                    ancillary.depositSpecArm()
                            )
                    );
                }

                driveActions.add(drive.alignRung());

            } else if (!currentGamepad2.triangle && previousGamepad2.triangle) {
                status.position = samplePosition.NONE;
                status.colour = Ancillary.sampleColour.NONE;
                ancillaryActions.clear();
                driveActions.clear();
                drive.kill();
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

        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            modifier = 0.25;
            completedAction = false;
            driveActions.clear();
            driveActions.add(drive.alignWall());
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
            status.position = samplePosition.OUTTAKE;

            driveActions.clear();
            if (completedAction) {
                completedAction = false;
                ancillaryActions.add(
                        new SequentialAction(
                                ancillary.closeGrip(),
                                new ParallelAction(
                                    ancillary.outtakeTravelArm(),
                                        ancillary.travelExtendo()
                                )
                        )
                );
            } else {
                ancillaryActions.add(
                        homeAncillary
                );
            }
        }

        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            ancillaryActions.clear();
            driveActions.clear();
            ancillaryActions.add(new SequentialAction(
                    new ParallelAction(
                        ancillary.intakeExtendo(),
                        ancillary.intakeLowerArmSafe()
                    ),
                    ancillary.expelAction()
                    )
            );
        } else if (!currentGamepad2.left_bumper && previousGamepad2.left_bumper) {
            ancillaryActions.clear();
            status.position = samplePosition.NONE;
            status.colour = Ancillary.sampleColour.NONE;
            ancillaryActions.add(new ParallelAction(
                    ancillary.travelExtendo(),
                    ancillary.intakeTravelArm(),
                    ancillary.wheelHaltAction()
                    )
            );
        }

        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            ancillaryActions.clear();
            driveActions.clear();
            modifier = 0.25;
            extendoControl = true;
            ancillaryActions.add(new SequentialAction(
                    new ParallelAction(
                            ancillary.intakeAction(),
                            ancillary.intakeLowerArm(),
                            lift.liftRetract(),
                            ancillary.outtakeTransferArm(),
                            ancillary.openGrip()
                    )
            ));
        } else if (previousGamepad2.dpad_up && !currentGamepad2.dpad_up) {
            status.position = samplePosition.INTAKE;
            ancillaryActions.clear();
            modifier = 1;
            extendoControl = false;
            ancillaryActions.add(
                    new ParallelAction(
                        ancillary.wheelHaltAction(),
                            ancillary.intakeTravelArm(),
                            ancillary.travelExtendo()
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
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        telemetry.addData("Sample colour", colour);
    }
}



