package org.firstinspires.ftc.teamcode.powercut.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;
import org.firstinspires.ftc.teamcode.powercut.hardware.SafeAncillary;

import java.util.ArrayList;
import java.util.List;


/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Disabled
@Autonomous(name = "4 Basket", preselectTeleOp = "DriveTeleOp", group = "Sample")
public class FourSampAuto extends OpMode {

    private Follower follower;
    private final Robot robot = new Robot();
    private SafeAncillary ancillary;
    private Lift lift;
    private LightSystem light;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private boolean readyToContinue = false;
    private boolean actionComplete = false;
    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7, 112, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(13.5, 129.5, Math.toRadians(-45));

    private final Pose intake1Pose1 = new Pose(20,121, Math.toRadians(0));
    private final Pose intake1Pose2 = new Pose(28,121, Math.toRadians(0));

    private final Pose intake2Pose1 = new Pose(20,129, Math.toRadians(0));
    private final Pose intake2Pose2 = new Pose(28,129, Math.toRadians(0));
    private final Pose intake3Pose1 = new Pose(20,128, Math.toRadians(20));
    private final Pose intake3Pose2 = new Pose(28,130, Math.toRadians(20));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Point parkControlPoint = new Point(60, 123);
    private final Pose parkPose = new Pose(60, 94, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, pickupIntake1, pickupIntake2, pickupIntake3, scorePickup1, scorePickup2, scorePickup3, park;
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */



        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose),new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(intake1Pose1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose1.getHeading())
                .build();

        pickupIntake1  = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose1), new Point(intake1Pose2)))
                .setLinearHeadingInterpolation(intake1Pose1.getHeading(), intake1Pose2.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose2), new Point(scorePose)))
                .setLinearHeadingInterpolation(intake1Pose2.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(intake2Pose1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose1.getHeading())
                .build();

        pickupIntake2  = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose1), new Point(intake2Pose2)))
                .setLinearHeadingInterpolation(intake2Pose1.getHeading(), intake2Pose2.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose2), new Point(scorePose)))
                .setLinearHeadingInterpolation(intake2Pose2.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(intake3Pose1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake3Pose1.getHeading())
                .build();

        pickupIntake3  = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose1), new Point(intake3Pose2)))
                .setLinearHeadingInterpolation(intake3Pose1.getHeading(), intake3Pose2.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose2), new Point(scorePose)))
                .setLinearHeadingInterpolation(intake3Pose2.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), parkControlPoint, new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                readyToContinue = false;
                actionComplete = false;

                runningActions.add(new SequentialAction(
                        new ParallelAction(
                                ancillary.closeGrip(),
                                lift.liftTopBasket()
                        ),
                        new InstantAction(() -> readyToContinue = true),
                        new InstantAction(() -> actionComplete = true)
                ));

                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && readyToContinue) {
                    lift.isLiftAvailable = true;
                    runningActions.clear();
                    /* Score Preload */
                    readyToContinue = false;
                    actionComplete = false;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    runningActions.add(new SequentialAction(
                            ancillary.depositSampArm(),
                            new SleepAction(0.2),
                            ancillary.openGrip(),
                            new SleepAction(0.5),

                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> lift.kill()),
                            new InstantAction(() -> follower.followPath(grabPickup1, true)),
                            new InstantAction(() -> setPathState(2)),

                            new ParallelAction(
                                    new SequentialAction(
                                            new SleepAction(1),
                                            lift.liftRetract()
                                    ),
                                    ancillary.outtakeTransferArm(),
                                    ancillary.intakeLowerArm(),
                                    ancillary.spinUpAction()
                            ),

                            new InstantAction(() -> actionComplete = true)
                    ));
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    readyToContinue = false;
                    actionComplete = false;
                    /* Score Sample */

                    runningActions.add(new SequentialAction(
                                new ParallelAction(
                                        ancillary.intakeLowerArm(),
                                        ancillary.intakeExtendo()
                                ),
                                new ParallelAction(
                                        ancillary.intakeAction(),
                                        new InstantAction(() -> follower.followPath(pickupIntake1, 0.75, true))
                                ),

                                new InstantAction(() -> readyToContinue = true),
                                new InstantAction(() -> actionComplete = true),
                                new InstantAction(() -> setPathState(3))
                            )
                    );
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    /* Score Sample */
                    readyToContinue = false;
                    actionComplete = false;

                    follower.followPath(scorePickup1);
                    setPathState(4);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    runningActions.add(new SequentialAction(
                            new ParallelAction(
                                    ancillary.outtakeTransferArm(),
                                    ancillary.transferExtendo(),
                                    ancillary.intakeTransferArm()
                            ),
                            new SleepAction(0.1),
                            ancillary.transferAction(),
                            new SleepAction(0.1),
                            ancillary.closeGrip(),
                            new SleepAction(0.1),
                            ancillary.clearanceExtendo(),
                            lift.liftTopBasket(),
                            ancillary.depositSampArm(),

                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> actionComplete = true),
                            new InstantAction(() -> setPathState(4))
                    ));

                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    /* Score Sample */
                    readyToContinue = false;
                    actionComplete = false;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    runningActions.add(new SequentialAction(
                            new SleepAction(0.2),
                            ancillary.openGrip(),
                            new SleepAction(0.2),

                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> follower.followPath(grabPickup2,  true)),
                            new InstantAction(() -> setPathState(5)),

                            new ParallelAction(
                                    new SequentialAction(
                                            new SleepAction(1),
                                        lift.liftRetract()
                                    ),
                                    ancillary.outtakeTransferArm(),
                                    ancillary.intakeLowerArm(),
                                    ancillary.spinUpAction()
                            ),
                            new InstantAction(() -> actionComplete = true)
                    ));
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy() && readyToContinue) {
                    lift.isLiftAvailable = true;
                    readyToContinue = false;
                    actionComplete = false;
                    /* Score Sample */

                    runningActions.add(new SequentialAction(
                            ancillary.intakeLowerArm(),
                            ancillary.intakeExtendo(),
                            new ParallelAction(
                                ancillary.intakeAction(),
                                new InstantAction(() -> follower.followPath(pickupIntake2, 0.75, true))
                            ),
                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> actionComplete = true),
                            new InstantAction(() -> setPathState(6))
                    ));

                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    readyToContinue = false;
                    actionComplete = false;

                    follower.followPath(scorePickup2);
                    setPathState(7);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    runningActions.add(new SequentialAction(
                            new ParallelAction(
                                    ancillary.outtakeTransferArm(),
                                    ancillary.transferExtendo(),
                                    ancillary.intakeTransferArm()
                            ),
                            new SleepAction(0.1),
                            ancillary.transferAction(),
                            new SleepAction(0.1),
                            ancillary.closeGrip(),
                            new SleepAction(0.1),
                            ancillary.clearanceExtendo(),
                            lift.liftTopBasket(),
                            ancillary.depositSampArm(),
                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> actionComplete = true),
                            new InstantAction(() -> setPathState(7))
                    ));
                }
                break;
            case 7:
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    readyToContinue = false;
                    actionComplete = false;

                    runningActions.add(new SequentialAction(
                            new SleepAction(0.5),
                            ancillary.openGrip(),
                            new SleepAction(0.5),
                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> follower.followPath(grabPickup3, true)),
                            new InstantAction(() -> setPathState(8)),
                            new ParallelAction(
                                    new SequentialAction(
                                            new SleepAction(1),
                                            lift.liftRetract()
                                    ),
                                    ancillary.outtakeTransferArm(),
                                    ancillary.intakeLowerArm(),
                                    ancillary.spinUpAction()
                            ),
                            new InstantAction(() -> actionComplete = true)
                    ));
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy() && readyToContinue) {
                    lift.isLiftAvailable = true;
                    readyToContinue = false;
                    actionComplete = false;
                    /* Score Sample */

                    runningActions.add(new SequentialAction(
                            ancillary.intakeLowerArm(),
                            ancillary.intakeExtendo(),
                            new ParallelAction(
                                    ancillary.intakeAction(),
                                    new InstantAction(() -> follower.followPath(pickupIntake3, 0.75, true))
                            ),
                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> actionComplete = true),
                            new InstantAction(() -> setPathState(9))
                    ));

                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    readyToContinue = false;
                    actionComplete = false;

                    follower.followPath(scorePickup3);
                    setPathState(10);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    runningActions.add(new SequentialAction(
                            new ParallelAction(
                                    ancillary.outtakeTransferArm(),
                                    ancillary.transferExtendo(),
                                    ancillary.intakeTransferArm()
                            ),
                            new SleepAction(0.1),
                            ancillary.transferAction(),
                            new SleepAction(0.1),
                            ancillary.closeGrip(),
                            new SleepAction(0.1),
                            ancillary.clearanceExtendo(),
                            lift.liftTopBasket(),
                            ancillary.depositSampArm(),
                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> actionComplete = true),
                            new InstantAction(() -> setPathState(7))
                    ));
                }
                break;
            case 10:
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    readyToContinue = false;
                    actionComplete = false;

                    runningActions.add(new SequentialAction(
                            new SleepAction(0.5),
                            ancillary.openGrip(),
                            new SleepAction(0.5),
                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> follower.followPath(park, true)),
                            new InstantAction(() -> setPathState(11)),
                            new ParallelAction(
                                    new SequentialAction(
                                            new SleepAction(1),
                                            lift.liftLevel1()
                                    ),
                                    ancillary.outtakeTransferArm(),
                                    ancillary.intakeTravelArm(),
                                    ancillary.clearanceExtendo()
                            ),
                            new InstantAction(() -> actionComplete = true)
                    ));
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {


                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();

        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        if (lift.isLiftAvailable && lift.liftStop.getState()) {
            lift.holdPosition();
        }

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Left Lift", lift.powerOutLeft);
        telemetry.addData("Right Lift", lift.powerOutRight);
        telemetry.addData("Left Lift", lift.leftLift.getCurrentPosition());
        telemetry.addData("Right Lift", lift.rightLift.getCurrentPosition());

        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Ready to continue", readyToContinue);

        telemetry.addData("Sample Colour", ancillary.getSampleColour());

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        Robot.pose = follower.getPose();
        Robot.heading = follower.getPose().getHeading();
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot.init(hardwareMap);
        lift = robot.getLift();
        ancillary = robot.getAncillary();
        light = robot.getLight();

        runningActions.add(new ParallelAction(
                ancillary.relaxGrip(),
                ancillary.clearanceExtendo(),
                ancillary.intakeTravelArm(),
                lift.liftRetractSensor(),
                ancillary.outtakeTravelArm(),
                new SequentialAction(
                        new SleepAction(5),
                        ancillary.closeGrip()
                )
        ));

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        light.partyWaves();
        telemetry.addLine("Ready");
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();

        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}