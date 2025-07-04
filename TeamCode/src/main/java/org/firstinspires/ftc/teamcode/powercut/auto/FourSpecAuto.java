package org.firstinspires.ftc.teamcode.powercut.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.hardware.lynx.LynxModule;
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

@Config
@Autonomous(name = "4 Spec", preselectTeleOp = "DriveTeleOp", group = "Specimen")
public class FourSpecAuto extends OpMode {

    List<LynxModule> allHubs;

    public static double speedModifer = 1.0;

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

    private boolean scored = false;
    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.5, 56.5, Math.toRadians(0));
    private final Pose score1Control = new Pose(16,75);

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose score1Pose1 = new Pose(40, 74, Math.toRadians(180));
    //   private final Pose score1Pose2 = new Pose(36.5, 72, Math.toRadians(180));

    private final Pose push1Pose1 = new Pose(58,26, Math.toRadians(0));
    private final Pose push1Control1 = new Pose(25, 76);
    private final Pose push1Control2 = new Pose(20, 17);
    private final Pose push1Control3 = new Pose(62, 40);

    private final Pose pickup1Pose1 = new Pose(20, 22, Math.toRadians(0));

    private final Pose push2Pose1 = new Pose(58, 14, Math.toRadians(0));
    private final Pose push2Control = new Pose(61.5, 30);




    private final Pose score2Control = new Pose(6,73.5);
    private final Pose score2Pose1 = new Pose(33, 72, Math.toRadians(180));
    private final Pose score2Pose2 = new Pose(40, 72, Math.toRadians(180));

    private final Pose pickup2Control = new Pose(32,22.5);

    private final Pose pickup2Pose1 = new Pose(18, 25.5, Math.toRadians(0));

    private final Pose score3Control = new Pose(6,71.5);
    private final Pose score3Pose1 = new Pose(33, 70, Math.toRadians(180));
    private final Pose score3Pose2 = new Pose(40, 70, Math.toRadians(180));

    private final Pose pickup3Control = new Pose(32,22.5);

    private final Pose pickup3Pose1 = new Pose(18, 25.5, Math.toRadians(0));

    private final Pose score4Control = new Pose(6,69.5);
    private final Pose score4Pose1 = new Pose(33, 68, Math.toRadians(180));
    private final Pose score4Pose2 = new Pose(40, 68, Math.toRadians(180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickupPose = new Pose(11.5, 24, Math.toRadians(0));


    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkMidPose = new Pose(29, 49.5, Math.toRadians(-135));
    private final Pose parkFinishPose = new Pose(20, 35, Math.toRadians(-135));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park;
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
                .addPath(new BezierCurve(new Point(startPose), new Point(score1Control), new Point(score1Pose1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), score1Pose1.getHeading())
                .setPathEndHeadingConstraint(500)
                .build();

//        score0Push = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(score1Pose1), new Point(score1Pose2)))
//                .setConstantHeadingInterpolation(score1Pose1.getHeading())
//                .setPathEndHeadingConstraint(250)
//                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score1Pose1), new Point(push1Control1), new Point(push1Control2), new Point(push1Control3), new Point(push1Pose1)))
                .setLinearHeadingInterpolation(score1Pose1.getHeading(), push1Pose1.getHeading())
                .addPath(new BezierLine(new Point(push1Pose1), new Point(pickup1Pose1)))
                .setLinearHeadingInterpolation(push1Pose1.getHeading(), pickup1Pose1.getHeading())
                .addPath(new BezierCurve(new Point(pickup1Pose1), new Point(push2Control), new Point(push2Pose1)))
                .setConstantHeadingInterpolation(pickup1Pose1.getHeading())
                .addPath(new BezierLine(new Point(push2Pose1), new Point(pickupPose)))
                .setLinearHeadingInterpolation(push2Pose1.getHeading(), pickupPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPose), new Point(score2Control), new Point(score2Pose1)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), score2Pose1.getHeading())
                .addPath(new BezierLine(new Point(score2Pose1), new Point(score2Pose2)))
                .setConstantHeadingInterpolation(score2Pose1.getHeading())
                .setPathEndHeadingConstraint(500)
                .build();


        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score2Pose1), new Point(pickup2Control), new Point(pickup2Pose1)))
                .setLinearHeadingInterpolation(score2Pose1.getHeading(), pickup2Pose1.getHeading())
                .addPath(new BezierLine(new Point(pickup2Pose1), new Point(pickupPose)))
                .setLinearHeadingInterpolation(pickup2Pose1.getHeading(), pickupPose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPose), new Point(score3Control), new Point(score3Pose1)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), score3Pose1.getHeading())
                .addPath(new BezierLine(new Point(score3Pose1), new Point(score3Pose2)))
                .setConstantHeadingInterpolation(score3Pose1.getHeading())
                .setPathEndHeadingConstraint(500)
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score3Pose1), new Point(pickup3Control), new Point(pickup3Pose1)))
                .setLinearHeadingInterpolation(score3Pose1.getHeading(), pickup3Pose1.getHeading())
                .addPath(new BezierLine(new Point(pickup3Pose1), new Point(pickupPose)))
                .setLinearHeadingInterpolation(pickup3Pose1.getHeading(), pickupPose.getHeading())
                .build();


        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPose), new Point(score4Control), new Point(score4Pose1)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), score4Pose1.getHeading())
                .addPath(new BezierLine(new Point(score4Pose1), new Point(score4Pose2)))
                .setConstantHeadingInterpolation(score4Pose1.getHeading())
                .setPathEndHeadingConstraint(500)
                .build();



        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score4Pose1), new Point(parkMidPose)))
                .setLinearHeadingInterpolation(score4Pose1.getHeading(), parkMidPose.getHeading())
                .addPath(new BezierLine(new Point(parkMidPose), new Point(parkFinishPose)))
                .setConstantHeadingInterpolation(parkFinishPose.getHeading())
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
                scored = false;

                runningActions.add(new SequentialAction(
                        new ParallelAction(
                                ancillary.closeGrip(),
                                lift.liftTopRungHigher(),
                                ancillary.alignMedianSpecArm()
                        ),
                        new InstantAction(() -> readyToContinue = true),
                        new InstantAction(() -> actionComplete = true)
                ));
                follower.followPath(scorePreload, speedModifer, true);
                setPathState(1);
                break;
            case 1:

                //else below runs before main if body

                if(!follower.isBusy() && readyToContinue) {
                    lift.isLiftAvailable = true;
                    /* Score Preload */
                    readyToContinue = false;
                    actionComplete = false;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> scored = true),
                            ancillary.openGrip(),
                            new InstantAction(() -> readyToContinue = true),
                             new InstantAction(() -> follower.followPath(grabPickup1, speedModifer, true)),
                             new InstantAction(() -> setPathState(2)),
                             new ParallelAction(
                                     new SequentialAction(
                                             new SleepAction(0.5),
                                             lift.liftRetract()
                                     ),
                                     ancillary.specIntakeArm(),
                                     ancillary.openGrip()
                             ),
                            new InstantAction(() -> actionComplete = true)
                    ));


                } else if (follower.getPose().getX() > 33  && !scored) {
                    runningActions.add(
                            new ParallelAction(
                            ancillary.looseCloseGrip(),
                            ancillary.depositMedianSpecArm()
                            )
                    );
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    readyToContinue = false;
                    actionComplete = false;
                    scored = false;
                    /* Score Sample */

                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                                ancillary.openGrip(),
                                ancillary.specIntakeArm(),
                                new SleepAction(0.5),
                                ancillary.closeGrip(),
                                new SleepAction(0.2),

                                new InstantAction(() -> follower.followPath(scorePickup1, speedModifer, true)),

                                ancillary.alignMedianSpecArm(),
                                lift.liftTopRungMedian(),

                                new InstantAction(() -> readyToContinue = true),
                                new InstantAction(() -> setPathState(3)),
                                new InstantAction(() -> actionComplete = true)
                            )
                    );


                }
                break;
            case 3:

                //else below runs before main body

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    /* Score Sample */
                    readyToContinue = false;
                    actionComplete = false;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> scored = true),
                            ancillary.openGrip(),

                            new InstantAction(() -> follower.followPath(grabPickup2, speedModifer, true)),

                            new ParallelAction(
                                    new SequentialAction(
                                            new SleepAction(1),
                                            lift.liftRetract()
                                    ),
                                    ancillary.specIntakeArm(),
                                    ancillary.openGrip()
                            ),

                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> setPathState(4)),
                            new InstantAction(() -> actionComplete = true)
                    ));


                } else if (follower.getPose().getX() > 32.5 && !scored) {
                    runningActions.add(
                            new ParallelAction(
                                    ancillary.looseCloseGrip(),
                                    ancillary.depositMedianSpecArm()
                            )
                    );
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    readyToContinue = false;
                    actionComplete = false;
                    scored = false;
                    /* Score Sample */

                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                                    ancillary.openGrip(),
                                    ancillary.specIntakeArm(),
                                    new SleepAction(0.5),
                                    ancillary.closeGrip(),
                                    new SleepAction(0.2),

                                    new InstantAction(() -> follower.followPath(scorePickup2, speedModifer, true)),

                                    ancillary.alignMedianSpecArm(),
                                    lift.liftTopRungMedian(),

                                    new InstantAction(() -> readyToContinue = true),
                                    new InstantAction(() -> setPathState(5)),
                                    new InstantAction(() -> actionComplete = true)
                            )
                    );
                }
                break;
            case 5:

                //else below runs previous to main body

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    /* Score Sample */
                    readyToContinue = false;
                    actionComplete = false;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> scored = true),
                            ancillary.openGrip(),

                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> follower.followPath(grabPickup3, speedModifer, true)),
                            new InstantAction(() -> setPathState(6)),

                            new ParallelAction(
                                    new SequentialAction(
                                            new SleepAction(0.5),
                                            lift.liftRetract()
                                    ),
                                    ancillary.specIntakeArm(),
                                    ancillary.openGrip()
                            ),
                            new InstantAction(() -> actionComplete = true)
                    ));


                } else if (follower.getPose().getX() > 31.5 && !scored) {
                    runningActions.add(
                            new ParallelAction(
                                    ancillary.looseCloseGrip(),
                                    ancillary.depositMedianSpecArm()
                            )
                    );
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    readyToContinue = false;
                    actionComplete = false;
                    scored = false;
                    /* Score Sample */

                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                                    ancillary.specIntakeArm(),
                                    new SleepAction(0.5),
                                    ancillary.closeGrip(),
                                    new SleepAction(0.2),

                                    new InstantAction(() -> follower.followPath(scorePickup3, speedModifer, true)),

                                    ancillary.alignMedianSpecArm(),

                                    new InstantAction(() -> readyToContinue = true),

                                    new InstantAction(() -> setPathState(7)),
                                    lift.liftTopRungMedian(),
                                    new InstantAction(() -> actionComplete = true)
                            )
                    );
                }
                break;
            case 7:
                if(!follower.isBusy() && actionComplete) {
                    lift.isLiftAvailable = true;
                    readyToContinue = false;
                    actionComplete = false;

                    runningActions.clear();
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> scored = true),
                            ancillary.openGrip(),
                            new InstantAction(() -> readyToContinue = true),
                            new InstantAction(() -> follower.followPath(park, true)),
                            new InstantAction(() -> setPathState(8)),
                            new ParallelAction(
                                    ancillary.intakeExpelArm(),
                                    ancillary.intakeExtendo(),
                                    new SequentialAction(
                                            new SleepAction(0.5),
                                            lift.liftRetract()
                                    ),
                                    ancillary.outtakeTransferArm()
                            ),
                            new InstantAction(() -> actionComplete = true)
                    ));

                } else if (follower.getPose().getX() > 31.5 && !scored) {
                    runningActions.add(
                            new ParallelAction(
                                    ancillary.looseCloseGrip(),
                                    ancillary.depositMedianSpecArm()
                            )
                    );
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
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

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

        telemetry.addData("Left Lift", lift.powerOutLeft);
        telemetry.addData("Right Lift", lift.powerOutRight);
        telemetry.addData("Left Lift", lift.leftLift.getCurrentPosition());
        telemetry.addData("Right Lift", lift.rightLift.getCurrentPosition());

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
                        ancillary.closeGrip(),
                        ancillary.transferExtendo()
                )
        ));

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

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