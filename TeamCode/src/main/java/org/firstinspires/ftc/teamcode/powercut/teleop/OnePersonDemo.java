package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;
import org.firstinspires.ftc.teamcode.powercut.hardware.SafeAncillary;
import org.firstinspires.ftc.teamcode.powercut.hardware.SafeAncillary.sampleColour;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class OnePersonDemo extends OpMode  {
    private final Robot robot = new Robot();
    //Hardware
    private SafeAncillary ancillary;
    private Lift lift;
    private Drivetrain drive;
    private LightSystem light;
    List<LynxModule> allHubs;
    Gamepad currentGamepad1 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();

    public static double thetaMultiplier = 0.6;

    //Localiser
    private MecanumDrive driveLoc = null;
    private Follower follower;

    private double heading = 0;

    //System monitoring
    private final ElapsedTime loopTimer = new ElapsedTime();

    private long startTime, cacheClearTime, refreshColorTime, gamepadDoneTime, actionsCompleteTime, localiserDoneTime, driveDone, ancillaryDone, telemetryDone, lightsDone;
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private Telemetry dashTelemetry = null;
    private List<Action> driveActions = new ArrayList<>();
    private List<Action> ancillaryActions = new ArrayList<>();

    private static boolean verbose = true;

    public enum samplePosition {
        INTAKE,
        TRAY,
        OUTTAKE,
        NONE
    }

    public class sampleStatus {
        sampleColour colour;
        samplePosition position;
    }

    public sampleStatus status = new sampleStatus();

    private boolean extendoControl = false;

    //Game monitoring
    boolean isHang = false;
    boolean completedAction = false;
    boolean liftTakeover = false;
    private double modifier = 1;
    ElapsedTime gametimer = new ElapsedTime();

    //control
    double x, y, theta;





    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = robot.getDrive();
        lift = robot.getLift();
        light = robot.getLight();
        ancillary = robot.getAncillary();

        heading = Robot.heading;

        Pose startPose = new Pose(-64, -64, heading);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashTelemetry = FtcDashboard.getInstance().getTelemetry();

        allHubs = hardwareMap.getAll(LynxModule.class);

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

        //follower.startTeleopDrive();

        ancillaryActions.add(new SequentialAction(
                ancillary.clearanceExtendo(),
                new ParallelAction(
                        ancillary.intakeTravelArm(),
                        ancillary.outtakeTransferArm(),
                        ancillary.closeGrip()
                ),
                lift.liftRetractSensor()
        ));
    }

    @Override
    public void loop() {
        startTime = System.nanoTime();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        cacheClearTime = System.nanoTime();

        //to run in background
        ancillary.refreshSampleColour();

        refreshColorTime = System.nanoTime();

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        gamepadDoneTime = System.nanoTime();

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

        actionsCompleteTime = System.nanoTime();

        follower.update();
        follower.drawOnDashBoard();



        Robot.pose = follower.getPose();
        Robot.heading = follower.getPose().getHeading();

        localiserDoneTime = System.nanoTime();

        doDrive();
        driveDone = System.nanoTime();
        doAncillary();
        ancillaryDone = System.nanoTime();
        lights();
        lightsDone = System.nanoTime();
        doTelemetry();

        loopTimer.reset();
        telemetry.update();
    }

    private void doDrive() {
        x = liftTakeover ? 0 : gamepad1.left_stick_x;
        y = liftTakeover ? 0 : -gamepad1.left_stick_y;
        theta = liftTakeover ? 0 : gamepad1.right_stick_x * thetaMultiplier;

        if (Math.abs(y) < 0.1 && Math.abs(x) > 0.9) {
            y = 0;
        }

        if (gamepad1.right_bumper) {
            modifier = 0.25;
        } else {
            modifier = 1;
        }

         if (gamepad1.touchpad) {
             follower.setPose(new Pose(0,0,0));
             gamepad1.rumble(200);
         }


        if (!isHang) {
            drive.setDrivetrainPowers(x, y, theta, modifier, Robot.heading, true);
        }
    }

    public void doAncillary() {
        if (gamepad1.left_trigger > 0.2) {
            liftTakeover = true;
            double liftPower = -gamepad1.left_stick_y;
            telemetry.addData("Lift under manual control, Power", liftPower);
            lift.isLiftAvailable = true;
            if (liftPower == 0) {
                lift.holdPosition();
            } else {
                lift.setLiftPower(liftPower);
            }
        } else if (lift.isLiftAvailable && lift.liftStop.getState() && !isHang) {
            liftTakeover = false;
            lift.holdPosition();
            if (verbose) {
                telemetry.addLine("Lift Hold active");
            }
        } else {
            liftTakeover = false;
        }





        if (extendoControl) {
            if (gamepad1.triangle) {
                ancillaryActions.add(ancillary.intakeExtendo());
            } else if (gamepad1.circle) {
                ancillaryActions.add(ancillary.halfExtendo());
            } else if (gamepad1.cross) {
                ancillaryActions.add(ancillary.transferExtendo());
            }
        } else {
            if (currentGamepad1.cross && !previousGamepad1.cross) {
                ancillaryActions.clear();
                ancillaryActions.add(ancillary.openGrip());
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
                                    new ParallelAction(
                                            ancillary.transferExtendo(),
                                            ancillary.intakeTransferArm()
                                    ),
                                    ancillary.transferAction(),
                                    new SleepAction(0.1),
                                    ancillary.closeGrip(),
                                    new SleepAction(0.5),
                                    lift.liftTopBasket(),
                                    ancillary.depositSampArm(),
                                    ancillary.clearanceExtendo(),
                                    ancillary.intakeTravelArm()

                            )
                    );
                }
            } else if (!currentGamepad1.cross && previousGamepad1.cross) {
                ancillaryActions.clear();
                ancillaryActions.add(
                        new SequentialAction(
                                ancillary.wheelHaltAction(),
                                ancillary.openGrip(),
                                new SleepAction(0.5),
                                ancillary.outtakeTransferArm(),
                                new SleepAction(1.5),
                                new ParallelAction(
                                        ancillary.closeGrip(),
                                        lift.liftRetract()
                                )
                        )
                );
            }
        }



//        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
//            completedAction = false;
//            ancillaryActions.clear();
//            ancillaryActions.add(new SequentialAction(
//                    new ParallelAction(
//                            lift.liftRetract(),
//                            ancillary.clearanceSpecExtendo(),
//                            ancillary.intakeTravelArm()
//                    ),
//                    new ParallelAction(
//                            ancillary.specIntakeArm(),
//                            ancillary.openGrip(),
//                            new InstantAction(() -> completedAction = true)
//                    ),
//                    ancillary.clearanceExtendo()
//            ));
//        } else if (!currentGamepad2.dpad_down && previousGamepad2.dpad_down) {
//            if (completedAction) {
//                completedAction = false;
//                status.position = samplePosition.OUTTAKE;
//                ancillaryActions.add(
//                        new SequentialAction(
//                                ancillary.closeGrip(),
//                                new SleepAction(0.3),
//                                new ParallelAction(
//                                    ancillary.outtakeLowerTravelArm(),
//                                        ancillary.clearanceExtendo()
//                                )
//                        )
//                );
//            } else {
//                ancillaryActions.add(
//                        new SequentialAction(
//                                ancillary.clearanceExtendo(),
//                                new ParallelAction(
//                                        ancillary.intakeTravelArm(),
//                                        ancillary.outtakeTransferArm(),
//                                        ancillary.closeGrip()
//                                )
//                        )
//                );
//            }
//        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            ancillaryActions.clear();
            driveActions.clear();
            ancillaryActions.add(new SequentialAction(
                    new ParallelAction(
                        ancillary.intakeExtendo(),
                        ancillary.intakeExpelArm()
                    ),
                    ancillary.spinOutAction(),
                    ancillary.expelAction()
                    )
            );
        } else if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
            ancillaryActions.clear();
            ancillaryActions.add(new ParallelAction(
                    ancillary.wheelHaltAction(),
                    ancillary.clearanceExtendo(),
                    ancillary.intakeTravelArm(),
                    ancillary.wheelHaltAction()
                    )
            );
        }

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
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
                            ancillary.openGrip(),
                            new SequentialAction(
                                    new SleepAction(0.5),
                                    ancillary.transferExtendo()
                            )

                    )
            ));
        } else if (previousGamepad1.dpad_up && !currentGamepad1.dpad_up) {
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



        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            ancillaryActions.clear();
            lift.kill();
            ancillaryActions.add(
                    lift.liftRetractSensor()
            );
            isHang = false;
        }



        if ((currentGamepad1.dpad_left && !previousGamepad1.dpad_left)) {
            ancillaryActions.clear();
            driveActions.clear();
            lift.kill();
            drive.kill();
            ancillary.relaxSystem();
            isHang = false;
        }

//        if (gamepad1.square || gamepad2.square || gamepad1.dpad_right) {
//            ancillaryActions.clear();
//            driveActions.clear();
//            lift.kill();
//            drive.kill();
//            isHang = false;
//            ancillaryActions.add(
//                    new SequentialAction(
//                            new SequentialAction(
//                                    ancillary.clearanceExtendo(),
//                                    new ParallelAction(
//                                            ancillary.intakeTravelArm(),
//                                            ancillary.outtakeTransferArm(),
//                                            ancillary.closeGrip()
//                                    )
//                            ),
//                            new ParallelAction(
//                                    lift.liftRetract()
//                            )
//                    )
//            );
//        }

    }

    public void doTelemetry() {
        telemetry.addData("L/R Lift Pos", "%d, %d", lift.leftLift.getCurrentPosition(), lift.rightLift.getCurrentPosition());

        if (verbose) {
            telemetry.addData("cacheTime", (cacheClearTime - startTime)/100);
            telemetry.addData("refreshColourTime", (refreshColorTime - cacheClearTime)/100);
            telemetry.addData("gamepadTime", (gamepadDoneTime - refreshColorTime)/100);
            telemetry.addData("actionTime", (actionsCompleteTime - gamepadDoneTime)/100);
            telemetry.addData("localiserTime", (localiserDoneTime - actionsCompleteTime)/100);
            telemetry.addData("driveTime", (driveDone - localiserDoneTime)/100);
            telemetry.addData("ancillaryTime", (ancillaryDone - driveDone)/100);
            telemetry.addData("lightsTime", (lightsDone - ancillaryDone)/100);

            telemetry.addData("Refresh rate", 1/loopTimer.seconds());
        }

        if (verbose) {
            telemetry.addData("L/R Lift Current", "%3.2f, %3.2f", lift.getLeftLiftCurrent(), lift.getRightLiftCurrent());

            telemetry.addData("intakeLeftArm isMoving", ancillary.intakeLeftArm.isMoving());
            telemetry.addData("intakeRightArm isMoving", ancillary.intakeRightArm.isMoving());
            telemetry.addData("upperLeftArm isMoving", ancillary.upperLeftArm.isMoving());
            telemetry.addData("upperRightArm isMoving", ancillary.upperRightArm.isMoving());
            telemetry.addData("extendoLeft isMoving", ancillary.extendoLeft.isMoving());
            telemetry.addData("extendoRight isMoving", ancillary.extendoRight.isMoving());
            telemetry.addData("grip isMoving", ancillary.grip.isMoving());
            telemetry.addData("Intake active", ancillary.intakeActive);

            telemetry.addData("Controls (X, Y Theta)", "%3.2f, %3.2f, %3.2f", x, y, theta);
            telemetry.addData("Colour", "%d, %d, %d", ancillary.colourSensor.red(), ancillary.colourSensor.green(), ancillary.colourSensor.blue());
//            telemetry.addData("Upper US Reads LR", "%d, %d", drive.leftUpperUS.getDistance(), drive.rightUpperUS.getDistance());
//            telemetry.addData("Lower Reads LR", "%5.1f, %5.1f", drive.getLowerLeftUS(), drive.getLowerRightUS());
//            telemetry.addData("ToF Reads LR", "%4.1f, %4.1f", drive.frontLeftToF.getDistance(DistanceUnit.MM), drive.frontRightToF.getDistance(DistanceUnit.MM));
        }


        telemetry.addData("x", Robot.pose.getX());
        telemetry.addData("y", Robot.pose.getY());
        telemetry.addData("heading (Rad)", Robot.pose.getHeading());
        telemetry.addData("heading (Deg)", Math.toDegrees(Robot.heading));

    }

    public void lights() {
        sampleColour sampleColour = ancillary.currentColour;
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
        } else if (gametimer.seconds() > 90 && gametimer.seconds() < 92) {
            light.redStrobe();
        } else if (gametimer.seconds() > 92) {
            light.redLarson();
        } else {
            light.greyLarson();
        }

        if (gametimer.seconds() > 90 && gametimer.seconds() < 90.2) {
            gamepad1.setLedColor(1.0,0.0,0.0, 30000);
            gamepad2.setLedColor(1.0,0.0,0.0, 30000);
            gamepad1.rumble(2000);
            gamepad2.rumble(2000);
        }

        telemetry.addData("Sample colour", colour);
    }
}



