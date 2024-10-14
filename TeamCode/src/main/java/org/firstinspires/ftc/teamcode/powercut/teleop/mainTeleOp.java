package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powercut.hardware.Arm;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class mainTeleOp extends OpMode {

    private final Arm arm = new Arm();
    private final Lift lift = new Lift();
    private final Drivetrain drive = new Drivetrain();
    private final LightSystem light = new LightSystem();

    private boolean authorised = false;
    private enum sequence {
        TopBasket,
        BottomBasket,
        TopRung,
        BottomRung,
        Intake
    }
    private sequence current = null;
    private enum basket {
        LiftExtend,
        ArmDeposit,
        Release
    }
    private basket basketCurrent = null;
    private enum rung {
        LiftExtend,
        LowerLift,
        Release
    }
    private rung rungCurrent = null;
    private enum intake {
        GripOpen,
        ArmDown,
        GripClosed,
        ArmUp
    }
    private intake intakeCurrent = null;

    // Actions
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        arm.init(hardwareMap);
        lift.init(hardwareMap);
        drive.init(hardwareMap);
        light.init(hardwareMap);


        light.confetti();
        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void start() {
        light.greyLarson();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double theta = gamepad1.right_stick_x;

        drive.setDrivetrainPowers(x, y, theta,1);

        ancillarySystemControl();

        telemetry.addData("Yaw", drive.getYaw());

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        telemetry.update();
    }

    private void ancillarySystemControl() {
        if (gamepad2.dpad_up) {
            runningActions.clear();
            current = sequence.TopBasket;
            basketCurrent = basket.LiftExtend;
            authorised = true;
        } else if (gamepad2.dpad_right) {
            runningActions.clear();
            current = sequence.BottomBasket;
            basketCurrent = basket.LiftExtend;
            authorised = true;
        } else if (gamepad2.dpad_left) {
            runningActions.clear();
            current = sequence.TopRung;
            rungCurrent = rung.LiftExtend;
            authorised = true;
        } else if (gamepad2.dpad_down) {
            runningActions.clear();
            current = sequence.BottomRung;
            rungCurrent = rung.LiftExtend;
            authorised = true;
        } else if (gamepad2.left_bumper) {
            runningActions.clear();
            current = sequence.Intake;
            intakeCurrent = intake.GripOpen;
            authorised = true;
        }

        if (gamepad2.cross) {
            authorised = true;
        }

        if (gamepad2.circle) {
            authorised = false;
            current = null;
            basketCurrent = null;
            rungCurrent = null;
            runningActions.clear();
        }

        if (gamepad2.triangle) {
            authorised = false;
            current = null;
            basketCurrent = null;
            rungCurrent = null;
            runningActions.clear();

            runningActions.add(new ParallelAction(
                    lift.liftRetract(),
                    arm.raiseArm()
                    )
            );
        }

        if (authorised) {
            if (current == sequence.TopBasket) {
                if (basketCurrent == basket.LiftExtend) {
                    runningActions.add(new SequentialAction(
                                    arm.raiseArm(),
                                    lift.liftTopBasket(),
                                    new InstantAction(() -> authorised = false),
                                    new InstantAction(() -> basketCurrent = basket.ArmDeposit)
                            )
                    );
                }

                else if (basketCurrent == basket.ArmDeposit) {
                    runningActions.add(new SequentialAction(
                            arm.depositArm(),
                            new InstantAction(() -> authorised = false),
                            new InstantAction(() -> basketCurrent = basket.Release)
                    ));
                }

                else if (basketCurrent == basket.Release) {
                    runningActions.add(
                            new ParallelAction(
                                new SequentialAction(
                                    arm.openGrip(),
                                    new SleepAction(0.1),
                                    new ParallelAction(
                                        arm.raiseArm(),
                                        lift.liftRetract()
                                    )
                                ),
                                    new InstantAction(() -> authorised = false),
                                    new InstantAction(() -> basketCurrent = null),
                                    new InstantAction(() -> current = null)
                            ));
                }

            }

            if (current == sequence.BottomBasket) {
                if (basketCurrent == basket.LiftExtend) {
                    runningActions.add(new SequentialAction(
                                    arm.raiseArm(),
                                    lift.liftBottomBasket(),
                                    new InstantAction(() -> authorised = false),
                                    new InstantAction(() -> basketCurrent = basket.ArmDeposit)
                            )
                    );
                }

                else if (basketCurrent == basket.ArmDeposit) {
                    runningActions.add(new SequentialAction(
                            arm.depositArm(),
                            new InstantAction(() -> authorised = false),
                            new InstantAction(() -> basketCurrent = basket.Release)
                    ));
                }

                else if (basketCurrent == basket.Release) {
                    runningActions.add(
                            new ParallelAction(
                                    new SequentialAction(
                                            arm.openGrip(),
                                            new SleepAction(0.1),
                                            new ParallelAction(
                                                    arm.raiseArm(),
                                                    lift.liftRetract()
                                            )
                                    ),
                                    new InstantAction(() -> authorised = false),
                                    new InstantAction(() -> basketCurrent = null),
                                    new InstantAction(() -> current = null)
                            ));
                }

            }

            if (current == sequence.TopRung) {
                if (rungCurrent == rung.LiftExtend) {
                    runningActions.add(new SequentialAction(
                            arm.raiseArm(),
                            lift.liftTopRung(),
                            arm.depositArm(),
                            new InstantAction(() -> authorised = false),
                            new InstantAction(() -> rungCurrent = rung.LowerLift)
                            ));
                }

                if (rungCurrent == rung.LowerLift) {
                    runningActions.add(new SequentialAction(
                            lift.liftTopRungAttached(),
                            new InstantAction(() -> authorised = false),
                            new InstantAction(() -> rungCurrent = rung.Release)
                    ));
                }

                if (rungCurrent == rung.Release) {
                    runningActions.add(new SequentialAction(
                            arm.openGrip(),
                            new SleepAction(0.1),
                            new ParallelAction(
                                   arm.raiseArm(),
                                   lift.liftRetract(),
                                    new InstantAction(() -> authorised = false),
                                    new InstantAction(() -> rungCurrent = null),
                                    new InstantAction(() -> current = null)
                            )
                    ));
                }
            }

            if (current == sequence.BottomRung) {
                if (rungCurrent == rung.LiftExtend) {
                    runningActions.add(new SequentialAction(
                            arm.raiseArm(),
                            lift.liftBottomRung(),
                            arm.depositArm(),
                            new InstantAction(() -> authorised = false),
                            new InstantAction(() -> rungCurrent = rung.LowerLift)
                    ));
                }

                else if (rungCurrent == rung.LowerLift) {
                    runningActions.add(new SequentialAction(
                            lift.liftBottomRungAttached(),
                            new InstantAction(() -> authorised = false),
                            new InstantAction(() -> rungCurrent = rung.Release)
                    ));
                }

                else if (rungCurrent == rung.Release) {
                    runningActions.add(new SequentialAction(
                            arm.openGrip(),
                            new SleepAction(0.1),
                            new ParallelAction(
                                    arm.raiseArm(),
                                    lift.liftRetract(),
                                    new InstantAction(() -> authorised = false),
                                    new InstantAction(() -> rungCurrent = null),
                                    new InstantAction(() -> current = null)
                            )
                    ));
                }
            }

            if (current == sequence.Intake) {
                if (intakeCurrent == intake.GripOpen) {
                    runningActions.add(new SequentialAction(
                    new ParallelAction(
                            lift.liftRetract(),
                            arm.raiseArm(),
                            arm.openGrip()
                    ),
                            new InstantAction(() -> authorised = false),
                            new InstantAction(() -> intakeCurrent = intake.ArmDown)
                    ));
                }

                else if (intakeCurrent == intake.ArmDown) {
                    runningActions.add(new SequentialAction(
                            arm.lowerArm(),
                            new InstantAction(() -> authorised = false),
                            new InstantAction(() -> intakeCurrent = intake.GripClosed)
                    ));
                }

                else if (intakeCurrent == intake.GripClosed) {
                    runningActions.add(new SequentialAction(
                            arm.closeGrip(),
                            new InstantAction(() -> authorised = false),
                            new InstantAction(() -> intakeCurrent = intake.ArmUp)
                    ));
                }

                else if (intakeCurrent == intake.ArmUp) {
                    runningActions.add(new SequentialAction(
                            arm.raiseArm(),
                            new InstantAction(() -> authorised = false),
                            new InstantAction(() -> intakeCurrent = null),
                            new InstantAction(() -> current = null)
                    ));
                }
            }
        }
    }

}
