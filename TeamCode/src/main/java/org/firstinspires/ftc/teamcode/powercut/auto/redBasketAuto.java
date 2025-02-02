package org.firstinspires.ftc.teamcode.powercut.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.powercut.hardware.Intake;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Outtake;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class redBasketAuto extends OpMode {
    private MecanumDrive drive;
    private final Outtake outtake = new Outtake();
    private final Lift lift = new Lift();
    private final LightSystem light = new LightSystem();
    private final Intake intake = new Intake();

    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private int sequence = 1;
    private boolean start = false;


    private Action toSpike1;
    private Action intakeAction1;
    private Action toBasket1;
    private Action transferAction1;
    private Action topBasketDeposit1;
    private Action toSpike2;
    private Action intakeAction2;
    private Action toBasket2;
    private Action transferAction2;
    private Action topBasketDeposit2;
    private Action toSpike3;
    private Action intakeAction3;
    private Action toBasket3;
    private Action transferAction3;
    private Action topBasketDeposit3;


//    private Action intakeSpec;
//    private Action topRungDeposit;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0 , 0, Math.toRadians(0)));
        outtake.init(hardwareMap);
        lift.init(hardwareMap);
        light.init(hardwareMap);
        intake.init(hardwareMap);



//        intakeSpec = new SequentialAction(
//                new ParallelAction(
//                        lift.liftRetract(),
//                        outtake.specIntakeArm()
//                ),
//                outtake.openGrip(),
//                new SleepAction(1),
//                outtake.closeGrip(),
//                outtake.travelArm()
//        );
//
//        topRungDeposit = new SequentialAction(
//                lift.liftTopRung(),
//                outtake.depositArm(),
//                outtake.relaxGrip(),
//                lift.liftTopRungAttached(),
//                outtake.openGrip(),
//                new ParallelAction(
//                        outtake.transferArm(),
//                        new SequentialAction(
//                                new SleepAction(0.5),
//                                lift.liftRetract()
//                        ),
//                        outtake.closeGrip()
//
//                )
//        );



        toSpike1 = new ParallelAction(
            intake.lowerArmSafe(),
                intake.spinUpAction(),
            drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(20, 20))
                    .build()
        );

        intakeAction1 = new RaceAction(
                new ParallelAction(
                        intake.lowerArm(),
                        intake.intakeExtendo(),
                        outtake.transferArm(),
                        outtake.openGrip(),
                        lift.liftRetract(),
                        new SleepAction(10)
                ),
                intake.intakeAction()
        );

        toBasket1 = drive.actionBuilder(new Pose2d(20,20, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-10, 20), Math.toRadians(-45.00))
                .build();

        transferAction1 = new SequentialAction(
                new ParallelAction(
                        intake.travelArm(),
                        intake.transfer1Extendo()
                ),
                intake.transferArm(),
                intake.transfer2Extendo(),
                intake.transferAction(),
                outtake.closeGrip(),
                lift.liftTopBasket()
        );

        topBasketDeposit1 = new SequentialAction(
                outtake.depositArm(),
                outtake.openGrip()
        );

        toSpike2 = new ParallelAction(
                outtake.closeGrip(),
                outtake.transferArm(),
                lift.liftRetract(),
                intake.lowerArmSafe(),
                intake.spinUpAction(),
                drive.actionBuilder(new Pose2d(-10, 20, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(20, 10), Math.toRadians(0.00))
                .build()
        );

        intakeAction2 = new SequentialAction(
                new RaceAction(
                        new ParallelAction(
                                intake.lowerArm(),
                                intake.intakeExtendo(),
                                outtake.transferArm(),
                                outtake.openGrip(),
                                lift.liftRetract(),
                                new SleepAction(10)
                        ),
                        intake.intakeAction()
                )
        );

        toBasket2 = drive.actionBuilder(new Pose2d(20, 10, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-10, 20), Math.toRadians(-45.00))
                .build();

        transferAction2 = new SequentialAction(
                new ParallelAction(
                        intake.travelArm(),
                        intake.transfer1Extendo()
                ),
                intake.transferArm(),
                intake.transfer2Extendo(),
                intake.transferAction(),
                outtake.closeGrip(),
                lift.liftTopBasket()
        );

        topBasketDeposit2 = new SequentialAction(
                outtake.depositArm(),
                outtake.openGrip()
        );

        toSpike3 = new ParallelAction(
                outtake.closeGrip(),
                outtake.transferArm(),
                lift.liftRetract(),
                intake.lowerArmSafe(),
                intake.spinUpAction(),
                drive.actionBuilder(new Pose2d(-10, 20, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(20, 0), Math.toRadians(0.00))
                .build()
        );

        intakeAction3 = new SequentialAction(
                new RaceAction(
                        new ParallelAction(
                                intake.lowerArm(),
                                intake.intakeExtendo(),
                                outtake.transferArm(),
                                outtake.openGrip(),
                                lift.liftRetract(),
                                new SleepAction(10)
                        ),
                        intake.intakeAction()
                )
        );

        toBasket3 = drive.actionBuilder(new Pose2d(20, 0, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-10, 20), Math.toRadians(-45.00))
                .build();

        transferAction3 = new SequentialAction(
                new ParallelAction(
                        intake.travelArm(),
                        intake.transfer1Extendo()
                ),
                intake.transferArm(),
                intake.transfer2Extendo(),
                intake.transferAction(),
                outtake.closeGrip(),
                lift.liftTopBasket()
        );

        topBasketDeposit3 = new SequentialAction(
                outtake.depositArm(),
                outtake.openGrip(),
                new ParallelAction(
                        outtake.closeGrip(),
                        outtake.transferArm(),
                        lift.liftRetract()
                )
        );

    }

    @Override
    public void start() {
        runningActions.add(new SequentialAction(
                toSpike1,
                intakeAction1,
                new ParallelAction(
                        transferAction1,
                        toBasket1
                ),
                topBasketDeposit1,
                new InstantAction(() -> sequence = 1),
                new InstantAction(() -> start = true)
        ));





    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if (start) {
            switch (sequence) {
                case 1:
                    start = false;
                    runningActions.add(new SequentialAction(
                            toSpike2,
                            intakeAction2,
                            new ParallelAction(
                                    transferAction2,
                                    toBasket2
                            ),
                            topBasketDeposit2,
                            new InstantAction(() -> sequence = 2),
                            new InstantAction(() -> start = true)
                    ));
                    break;
                case 2:
                    start = false;
                    runningActions.add(new SequentialAction(
                            toSpike3,
                            intakeAction3,
                            new ParallelAction(
                                    transferAction3,
                                    toBasket3
                            ),
                            topBasketDeposit3
                    ));
                    break;
            }

        }

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
    }
}
