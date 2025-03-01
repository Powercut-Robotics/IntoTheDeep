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

import org.firstinspires.ftc.teamcode.powercut.hardware.Ancillary;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class RRredBasketAuto extends OpMode {
    private MecanumDrive drive;
    private Robot robot = new Robot();
    private Ancillary ancillary;
    private Lift lift;
    private LightSystem light;

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
    private Action toBasket0;
    private Action topBasketDeposit0;
    private Action topBasketPrep0;


    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-42, 64, Math.toRadians(90)));
        robot.init(hardwareMap);
        ancillary = robot.getAncillary();
        lift = robot.getLift();
        light = robot.getLight();

        toBasket0 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45.00))
                .build();

        topBasketPrep0 = new ParallelAction(
                lift.liftTopBasket(),
                ancillary.outtakeTravelArm(),
                ancillary.closeGrip()
        );

        topBasketDeposit0 = new SequentialAction(
                ancillary.depositSampArm(),
                ancillary.openGrip()
        );

        toSpike1 = new ParallelAction(
            ancillary.intakeLowerArmSafe(),
                ancillary.spinUpAction(),
            drive.actionBuilder(new Pose2d(-54,-54, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(-52, -44), Math.toRadians(90))
                    .build()
        );

        intakeAction1 = new RaceAction(
                new ParallelAction(
                        ancillary.intakeLowerArm(),
                        ancillary.intakeExtendo(),
                        ancillary.intakeTransferArm(),
                        ancillary.openGrip(),
                        lift.liftRetract(),
                        new SleepAction(10)
                ),
                ancillary.intakeAction()
        );

        toBasket1 = drive.actionBuilder(new Pose2d(-52,-44, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45.00))
                .build();

        transferAction1 = new SequentialAction(
                new ParallelAction(
                        ancillary.intakeTravelArm(),
                        ancillary.travelExtendo()
                ),
                ancillary.intakeTransferArm(),
                ancillary.transferExtendo(),
                ancillary.transferAction(),
                ancillary.closeGrip(),
                lift.liftTopBasket()
        );

        topBasketDeposit1 = new SequentialAction(
                ancillary.depositSampArm(),
                ancillary.openGrip()
        );

        toSpike2 = new ParallelAction(
                ancillary.closeGrip(),
                ancillary.intakeTransferArm(),
                lift.liftRetract(),
                ancillary.intakeLowerArmSafe(),
                ancillary.spinUpAction(),
                drive.actionBuilder(new Pose2d(-10, 20, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-60, -44), Math.toRadians(90))
                .build()
        );

        intakeAction2 = new SequentialAction(
                new RaceAction(
                        new ParallelAction(
                                ancillary.intakeLowerArm(),
                                ancillary.intakeExtendo(),
                                ancillary.intakeTransferArm(),
                                ancillary.openGrip(),
                                lift.liftRetract(),
                                new SleepAction(10)
                        ),
                        ancillary.intakeAction()
                )
        );

        toBasket2 = drive.actionBuilder(new Pose2d(-60, -44, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45.00))
                .build();

        transferAction2 = new SequentialAction(
                new ParallelAction(
                        ancillary.intakeTravelArm(),
                        ancillary.travelExtendo()
                ),
                ancillary.intakeTransferArm(),
                ancillary.transferExtendo(),
                ancillary.transferAction(),
                ancillary.closeGrip(),
                lift.liftTopBasket()
        );

        topBasketDeposit2 = new SequentialAction(
                ancillary.depositSampArm(),
                ancillary.openGrip()
        );

        toSpike3 = new ParallelAction(
                ancillary.closeGrip(),
                ancillary.intakeTransferArm(),
                lift.liftRetract(),
                ancillary.intakeLowerArmSafe(),
                ancillary.spinUpAction(),
                drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-68, -44), Math.toRadians(90))
                .build()
        );

        intakeAction3 = new SequentialAction(
                new RaceAction(
                        new ParallelAction(
                                ancillary.intakeLowerArm(),
                                ancillary.intakeExtendo(),
                                ancillary.intakeTransferArm(),
                                ancillary.openGrip(),
                                lift.liftRetract(),
                                new SleepAction(10)
                        ),
                        ancillary.intakeAction()
                )
        );

        toBasket3 = drive.actionBuilder(new Pose2d(-68, -44, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45.00))
                .build();

        transferAction3 = new SequentialAction(
                new ParallelAction(
                        ancillary.intakeTravelArm(),
                        ancillary.travelExtendo()
                ),
                ancillary.intakeTransferArm(),
                ancillary.transferExtendo(),
                ancillary.transferAction(),
                ancillary.closeGrip(),
                lift.liftTopBasket()
        );

        topBasketDeposit3 = new SequentialAction(
                ancillary.depositSampArm(),
                ancillary.openGrip(),
                new ParallelAction(
                        ancillary.closeGrip(),
                        ancillary.intakeTransferArm(),
                        lift.liftRetract()
                )
        );

    }

    @Override
    public void start() {
        runningActions.add(new SequentialAction(
                new ParallelAction(
                        toBasket0,
                        topBasketPrep0
                ),
                topBasketDeposit0,
                new InstantAction(() -> sequence = 0),
                new InstantAction(() -> start = true)
        ));

    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if (start) {
            switch (sequence) {
                case 0:
                    start = false;
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
