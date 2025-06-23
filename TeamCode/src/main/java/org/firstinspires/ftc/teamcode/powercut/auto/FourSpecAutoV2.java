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

@Autonomous(name = "4 Spec v2", preselectTeleOp = "DriveTeleOp", group = "Specimen")
public class FourSpecAutoV2 extends OpMode {

    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private boolean readyToContinue = false;

    private Follower follower;

    private final Robot robot = new Robot();
    private SafeAncillary ancillary;
    private Lift lift;
    private LightSystem light;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 52.5, Math.toRadians(180));

    /** pickup pose global def **/

    private final Pose pickupPose = new Pose(12, 20.5, Math.toRadians(0));

    /** Scoring Pose of our robot. */
    private final Pose scorePreloadControl = new Pose(15, 72);
    private final Pose scorePreloadFinishPose = new Pose(38, 70, Math.toRadians(180));

    /** Push Points */
    private final Pose push1Control1 = new Pose(21, 23);
    private final Pose push1Control2 = new Pose(63.5, 37);
    private final Pose push1MidPose = new Pose(58, 20, Math.toRadians(0));
    private final Pose push1FinishPose = new Pose(12, 20.5, Math.toRadians(0));

    private final Pose push2Contol1 = new Pose(41.5, 18);
    private final Pose push2Control2 = new Pose(62.5, 27.5);
    private final Pose push2MidPose = new Pose(58, 69, Math.toRadians(0));
    private final Pose push2FinishPose = pickupPose;

    /** Score pickup 1 */
    private final Pose score1Control = new Pose(13.5, 69);
    private final Pose score1MidPose = new Pose(34, 68, Math.toRadians(180));
    private final Pose score1FinishPose = new Pose(38, 68, Math.toRadians(180));
    private final Pose pickup2Pose = pickupPose;

    /** Score pickup 2 */
    private final Pose score2Control = new Pose(13.5, 67);
    private final Pose score2MidPose = new Pose(34, 66, Math.toRadians(180));
    private final Pose score2FinishPose = new Pose(38, 66, Math.toRadians(180));
    private final Pose pickup3Pose = pickupPose;


    /** Score pickup 3 */
    private final Pose score3Control = new Pose(13.5, 65);
    private final Pose score3MidPose = new Pose(34, 64, Math.toRadians(180));
    private final Pose score3FinishPose = new Pose(38, 64, Math.toRadians(180));


    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(15, 26.5, Math.toRadians(-120));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */


    /* These are our Paths and PathChains that we will define in buildPaths() */

    private PathChain scorePreload, pushAndGrabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park;

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

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scorePreloadControl), new Point(scorePreloadFinishPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadFinishPose.getHeading())
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushAndGrabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreloadFinishPose), new Point(push1Control1), new Point(push1Control2), new Point(push1MidPose)))
                .setLinearHeadingInterpolation(scorePreloadFinishPose.getHeading(), push1MidPose.getHeading())
                .addPath(new BezierLine(new Point(push1MidPose), new Point(push1FinishPose)))
                .setLinearHeadingInterpolation(push1MidPose.getHeading(), push1FinishPose.getHeading())
                .addPath(new BezierCurve(new Point(push1FinishPose), new Point(push2Contol1), new Point(push2Control2), new Point(push2MidPose)))
                .setLinearHeadingInterpolation(push1FinishPose.getHeading(), push2MidPose.getHeading())
                .addPath(new BezierLine(new Point(push2MidPose), new Point(push2FinishPose)))
                .setLinearHeadingInterpolation(push2MidPose.getHeading(), push2FinishPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(push2FinishPose), new Point(score1Control), new Point(score1MidPose)))
                .setLinearHeadingInterpolation(push2FinishPose.getHeading(), score1MidPose.getHeading())
                .addPath(new BezierLine(new Point(score1MidPose), new Point(score1FinishPose)))
                .setLinearHeadingInterpolation(score1MidPose.getHeading(), score1FinishPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1FinishPose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(score1FinishPose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup2Pose), new Point(score2Control), new Point(score2MidPose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), score2MidPose.getHeading())
                .addPath(new BezierLine(new Point(score2MidPose), new Point(score2FinishPose)))
                .setLinearHeadingInterpolation(score2MidPose.getHeading(), score2FinishPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2FinishPose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(score2FinishPose.getHeading(), pickup3Pose.getHeading())
                .build();


        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup3Pose), new Point(score3Control), new Point(score3MidPose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), score3MidPose.getHeading())
                .addPath(new BezierLine(new Point(score3MidPose), new Point(score3FinishPose)))
                .setLinearHeadingInterpolation(score3MidPose.getHeading(), score3FinishPose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score3FinishPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(score3FinishPose.getHeading(), parkPose.getHeading())
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                readyToContinue = false;
                runningActions.clear();
                runningActions.add(new SequentialAction(
                                new ParallelAction(
                                        ancillary.closeGrip(),
                                        ancillary.depositHigherSpecArm(),
                                        lift.liftTopRungLowest()
                                ),
                                new InstantAction(() -> readyToContinue = true)
                        ));
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                if (follower.getPose().getX() > 32 && follower.isBusy()) {
                    runningActions.add(ancillary.relaxGrip());
                } else if (!follower.isBusy() && readyToContinue) {
                    readyToContinue = false;
                    /* Score Preload */
                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                            ancillary.lowSpecIntakeArm(),
                            ancillary.openGrip(),
                            new SleepAction(0.5),
                            lift.liftRetract(),
                            new InstantAction(() -> readyToContinue = true)

                    ));
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pushAndGrabPickup1, true);

                    setPathState(2);
                 }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy() && readyToContinue) {
                    readyToContinue = false;
                    /* Grab Sample */
                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                            new SleepAction(0.5),
                            ancillary.closeGrip(),
                            new SleepAction(0.2),
                            new InstantAction(() -> follower.followPath(scorePickup1,true)),
                            new ParallelAction(
                                    ancillary.depositHigherSpecArm(),
                                    lift.liftTopRungLowest()
                            )
                    ));
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > 32 && follower.isBusy()) {
                    runningActions.add(ancillary.relaxGrip());
                } else if (!follower.isBusy() && readyToContinue) {
                    readyToContinue = false;
                    /* Score Spec */
                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                            ancillary.lowSpecIntakeArm(),
                            ancillary.openGrip(),
                            new SleepAction(0.5),
                            lift.liftRetract(),
                            new InstantAction(() -> readyToContinue = true)

                    ));
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy() && readyToContinue) {
                    readyToContinue = false;
                    /* Grab Sample */
                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                            new SleepAction(0.5),
                            ancillary.closeGrip(),
                            new SleepAction(0.2),
                            new InstantAction(() -> follower.followPath(scorePickup1,true)),
                            new ParallelAction(
                                    ancillary.depositHigherSpecArm(),
                                    lift.liftTopRungLowest()
                            )
                    ));

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if (follower.getPose().getX() > 32 && follower.isBusy()) {
                        runningActions.add(ancillary.relaxGrip());
                    } else if (!follower.isBusy() && readyToContinue) {
                        readyToContinue = false;
                        /* Score Spec */
                        runningActions.clear();
                        runningActions.add(new SequentialAction(
                                ancillary.lowSpecIntakeArm(),
                                ancillary.openGrip(),
                                new SleepAction(0.5),
                                lift.liftRetract(),
                                new InstantAction(() -> readyToContinue = true)

                        ));
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup3, true);
                        setPathState(6);
                    }

                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy() && readyToContinue) {
                    readyToContinue = false;
                    /* Grab Sample */
                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                            new SleepAction(0.5),
                            ancillary.closeGrip(),
                            new SleepAction(0.2),
                            new InstantAction(() -> follower.followPath(scorePickup1,true)),
                            new ParallelAction(
                                    ancillary.depositHigherSpecArm(),
                                    lift.liftTopRungLowest()
                            )
                    ));

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > 32 && follower.isBusy()) {
                    runningActions.add(ancillary.relaxGrip());
                } else if (!follower.isBusy() && readyToContinue) {
                    readyToContinue = false;
                    /* Score Spec */
                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                            ancillary.lowSpecIntakeArm(),
                            ancillary.openGrip(),
                            new SleepAction(0.5),
                            new ParallelAction(
                                    lift.liftRetract(),
                                    ancillary.outtakeLowerTravelArm()
                            ),
                            new InstantAction(() -> readyToContinue = true)

                    ));
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

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

        if (follower.isRobotStuck()) {
            light.redStrobe();
        } else if (follower.isBusy()) {
            light.partyWaves();
        } else {
            light.greenWaves();
        }

        // These loop the movements of the robot
        follower.update();
        follower.drawOnDashBoard();
        autonomousPathUpdate();


        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Ready to continue", readyToContinue);

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
                ancillary.outtakeLowerTravelArm(),
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
