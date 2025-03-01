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
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.powercut.hardware.Ancillary;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;

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

@Autonomous(name = "4 Spec", group = "Specimen")
public class PedroSpecAuto extends OpMode {

    private Follower follower;
    private final Robot robot = new Robot();
    private Ancillary ancillary;
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
    private final Pose startPose = new Pose(7, 50, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose score1Pose = new Pose(38, 58, Math.toRadians(180));

    private final Pose push1Pose = new Pose(58,19, Math.toRadians(0));
    private final Pose push1Control1 = new Pose(27,16);
    private final Pose push1Control2 = new Pose(52,43);

    private final Pose score2Pose = new Pose(38, 60, Math.toRadians(180));
    private final Pose push2Pose = new Pose(58,7,Math.toRadians(0));
    private final Pose push2Control1 = new Pose(22,18);
    private final Pose push2Control2 = new Pose(61,33);

    private final Pose score3Pose = new Pose(38, 62, Math.toRadians(180));
    private final Pose score4Pose = new Pose(38, 64, Math.toRadians(180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickupPose = new Pose(10, 19, Math.toRadians(0));


    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(7.5, 19, Math.toRadians(0));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, push1, push2;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park;
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

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(score1Pose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score1Pose), new Point(push1Control1), new Point(push1Control2), new Point(push1Pose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), push1Pose.getHeading())
                .addPath(new BezierLine(new Point(push1Pose), new Point(pickupPose)))
                .setLinearHeadingInterpolation(push1Pose.getHeading(), pickupPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose), new Point(score2Pose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), score2Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score2Pose), new Point(push2Control1), new Point(push2Control2), new Point(push2Pose)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), push2Pose.getHeading())
                .addPath(new BezierLine(new Point(push2Pose), new Point(pickupPose)))
                .setLinearHeadingInterpolation(push2Pose.getHeading(), pickupPose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), score3Pose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score3Pose), new Point(pickupPose)))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), pickupPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose), new Point(score4Pose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), score4Pose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score4Pose), new Point(parkPose)))
                .setLinearHeadingInterpolation(score4Pose.getHeading(), parkPose.getHeading())
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
                                lift.liftTopRung(),
                                ancillary.depositSpecArm()
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
                    /* Score Preload */
                    readyToContinue = false;
                    actionComplete = false;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    runningActions.add(new SequentialAction(
                            lift.liftTopRungAttached(),
                            ancillary.openGrip(),
                            new InstantAction(() -> readyToContinue = true),
                             new ParallelAction(
                                     lift.liftRetract(),
                                     ancillary.specIntakeArm()
                             ),
                            new InstantAction(() -> actionComplete = true)
                    ));

                    if (readyToContinue) {
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && actionComplete) {
                    readyToContinue = false;
                    actionComplete = false;
                    /* Score Sample */

                    runningActions.add(new SequentialAction(
                                ancillary.closeGrip(),
                                ancillary.depositSpecArm(),
                                new InstantAction(() -> readyToContinue = true),
                                lift.liftTopRung(),
                                new InstantAction(() -> actionComplete = true)
                            )
                    );

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (readyToContinue) {
                        follower.followPath(scorePickup1, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && actionComplete) {
                    /* Score Sample */
                    readyToContinue = false;
                    actionComplete = false;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    runningActions.add(new SequentialAction(
                            lift.liftTopRungAttached(),
                            ancillary.openGrip(),
                            new InstantAction(() -> readyToContinue = true),
                            new ParallelAction(
                                    lift.liftRetract(),
                                    ancillary.specIntakeArm()
                            ),
                            new InstantAction(() -> actionComplete = true)
                    ));

                    if (readyToContinue) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup2, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy() && actionComplete) {
                    readyToContinue = false;
                    actionComplete = false;
                    /* Score Sample */

                    runningActions.add(new SequentialAction(
                            new SleepAction(1),
                                    ancillary.closeGrip(),
                                    ancillary.depositSpecArm(),
                                    new InstantAction(() -> readyToContinue = true),
                                    lift.liftTopRung(),
                                    new InstantAction(() -> actionComplete = true)
                            )
                    );

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (readyToContinue) {
                        follower.followPath(scorePickup2, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy() && actionComplete) {
                    readyToContinue = false;
                    actionComplete = false;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    runningActions.add(new SequentialAction(
                            lift.liftTopRungAttached(),
                            ancillary.openGrip(),
                            new InstantAction(() -> readyToContinue = true),
                            new ParallelAction(
                                    lift.liftRetract(),
                                    ancillary.specIntakeArm()
                            ),
                            new InstantAction(() -> actionComplete = true)
                    ));

                    if (readyToContinue) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(grabPickup2, true);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy() && actionComplete) {
                    readyToContinue = false;
                    actionComplete = false;
                    /* Score Sample */

                    runningActions.add(new SequentialAction(
                            new SleepAction(1),
                                    ancillary.closeGrip(),
                                    ancillary.depositSpecArm(),
                                    new InstantAction(() -> readyToContinue = true),
                                    lift.liftTopRung(),
                                    new InstantAction(() -> actionComplete = true)
                            )
                    );

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (readyToContinue) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                    }
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && actionComplete) {
                    /* Score Sample */

                    readyToContinue = false;
                    actionComplete = false;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    runningActions.add(new SequentialAction(
                            lift.liftTopRungAttached(),
                            ancillary.openGrip(),
                            new InstantAction(() -> readyToContinue = true),
                            new ParallelAction(
                                    lift.liftRetract(),
                                    ancillary.specIntakeArm()
                            ),
                            new InstantAction(() -> actionComplete = true)
                    ));

                    if (readyToContinue) {
                        follower.followPath(park, true);
                        setPathState(8);
                    }
                }
                break;
            case 8:
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

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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
                ancillary.travelExtendo(),
                ancillary.intakeTravelArm(),
                lift.liftRetract(),
                ancillary.outtakeTravelArm()
        ));

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        light.partyWaves();
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