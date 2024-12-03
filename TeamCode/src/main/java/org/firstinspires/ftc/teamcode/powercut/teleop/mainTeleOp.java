package org.firstinspires.ftc.teamcode.powercut.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powercut.hardware.Arm;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.settings;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class mainTeleOp extends OpMode {
    private final Arm arm = new Arm();
    private final Lift lift = new Lift();
    private final Drivetrain drive = new Drivetrain();
    private final LightSystem light = new LightSystem();

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double lastX = 0;
    private double lastY = 0;
    private double lastTheta = 0;

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
        Release
    }
    private basket basketCurrent = null;
    private enum rung {
        LiftExtend,
        LowerLift,
    }
    private rung rungCurrent = null;
    private enum intake {
        GripOpen,
        ArmDown,
        GripClosed,
        ArmUp
    }
    private intake intakeCurrent = null;

    private boolean authLast = false;

    // Actions
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        arm.init(hardwareMap);
        lift.init(hardwareMap);
        drive.init(hardwareMap);
        light.init(hardwareMap);

        drive.imu.resetYaw();

        light.confetti();
        telemetry.addLine("Initialised");
        telemetry.update();
    }

    @Override
    public void start() {
        light.greyLarson();
        loopTimer.reset();
    }

    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();
        double yaw = drive.getYaw();
        telemetry.addData("Yaw", yaw);
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double theta = gamepad1.right_stick_x;

        //cache motor control for faster loop times
        if ((Math.abs(x-lastX) > settings.driveCacheAmount) || (Math.abs(y-lastY) > settings.driveCacheAmount) || (Math.abs(theta-lastTheta) > settings.driveCacheAmount)){
            drive.setDrivetrainPowers(x, y, theta, 1);
            lastX = x;
            lastY = y;
            lastTheta = theta;
        }

        if (current == sequence.Intake) {
            gripBasedLightControl();
        }


        ancillarySystemControl();




        Arm.sampleColour sampleColour = arm.getSampleColour();
        String colour = "";
        if (sampleColour == Arm.sampleColour.RED) {
            light.red();
            colour = "Red";
        } else if (sampleColour == Arm.sampleColour.YELLOW) {
            light.yellow();
            colour = "Green";
        } else if (sampleColour == Arm.sampleColour.BLUE) {
            light.blue();
            colour = "Blue";
        } else {
            light.greyLarson();
            colour = "None";
        }

        telemetry.addLine("Sample: " + colour);

        telemetry.addData("Loop Timer", loopTimer.time(TimeUnit.MILLISECONDS));

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        telemetry.update();
        loopTimer.reset();
    }

    private void gripBasedLightControl() {
        Arm.sampleColour sampleColour = arm.getSampleColour();

        if (sampleColour == Arm.sampleColour.RED) {
            light.red();
        } else if (sampleColour == Arm.sampleColour.YELLOW) {
            light.yellow();
        } else if (sampleColour == Arm.sampleColour.BLUE) {
            light.blue();
        } else {
            light.greyLarson();
        }
    }

    private void ancillarySystemControl() {
        if (lift.isLiftAvailable) {
            lift.setLiftPower(settings.liftHoldPower);
        }

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


        if (gamepad2.cross && !authLast) {
            authorised = true;
        }
        authLast = gamepad2.cross;

        if (gamepad2.circle) {
            authorised = false;
            current = null;
            basketCurrent = null;
            rungCurrent = null;
            lift.isLiftAvailable = true;
            runningActions.clear();
        }

        if (gamepad2.triangle) {
            authorised = false;
            current = null;
            basketCurrent = null;
            rungCurrent = null;
            runningActions.clear();

            runningActions.add(
                    new SequentialAction(
                    new ParallelAction(
                        lift.liftRetract(),
                        arm.raiseArm()
                    ),
                            new InstantAction(() -> lift.isLiftAvailable = true)
                    )
            );
        }

        if (authorised) {
            authorised = false;
            if (current == sequence.TopBasket) {
                if (basketCurrent == basket.LiftExtend) {
                    runningActions.add(new SequentialAction(
                                    arm.raiseArm(),
                                    lift.liftTopBasket(),
                                    arm.depositArm(),
                                    new InstantAction(() -> basketCurrent = basket.Release)
                            )
                    );
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
                                    arm.depositArm(),
                                    new InstantAction(() -> basketCurrent = basket.Release)
                            )
                    );
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
                                    new InstantAction(() -> basketCurrent = null),
                                    new InstantAction(() -> current = null)
                            ));
                }

            }

            if (current == sequence.TopRung) {
                if (rungCurrent == rung.LiftExtend) {
                    runningActions.add(new SequentialAction(
                            arm.depositArm(),
                            lift.liftTopRung(),
                            new InstantAction(() -> rungCurrent = rung.LowerLift)
                            ));
                }

                if (rungCurrent == rung.LowerLift) {
                    runningActions.add(new SequentialAction(
                            lift.liftTopRungAttached(),
                            arm.openGrip(),
                            new SleepAction(0.1),
                            new ParallelAction(
                                    arm.raiseArm(),
                                    lift.liftRetract(),
                                    new InstantAction(() -> rungCurrent = null),
                                    new InstantAction(() -> current = null)
                            )
                    ));
                }


            }

            if (current == sequence.BottomRung) {
                if (rungCurrent == rung.LiftExtend) {
                    runningActions.add(new SequentialAction(
                            arm.depositArm(),
                            lift.liftBottomRung(),
                            new InstantAction(() -> rungCurrent = rung.LowerLift)
                    ));
                }

                else if (rungCurrent == rung.LowerLift) {
                    runningActions.add(new SequentialAction(
                            lift.liftBottomRungAttached(),
                            arm.openGrip(),
                            new SleepAction(0.1),
                            new ParallelAction(
                                    arm.raiseArm(),
                                    lift.liftRetract(),
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
                            new InstantAction(() -> intakeCurrent = intake.ArmDown)
                    ));
                }

                else if (intakeCurrent == intake.ArmDown) {
                    runningActions.add(new SequentialAction(
                            arm.lowerArm(),
                            new InstantAction(() -> intakeCurrent = intake.GripClosed)
                    ));
                }

                else if (intakeCurrent == intake.GripClosed) {
                    runningActions.add(new SequentialAction(
                            arm.closeGrip(),
                            new InstantAction(() -> intakeCurrent = intake.ArmUp)
                    ));
                }

                else if (intakeCurrent == intake.ArmUp) {
                    runningActions.add(new SequentialAction(
                            arm.raiseArm(),
                            new InstantAction(() -> intakeCurrent = null),
                            new InstantAction(() -> current = null)
                    ));
                }
            }
        }
    }

}
