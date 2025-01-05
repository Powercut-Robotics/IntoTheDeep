package org.firstinspires.ftc.teamcode.powercut.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.powercut.hardware.Lift;
import org.firstinspires.ftc.teamcode.powercut.hardware.LightSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Outtake;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class redBasketAuto extends OpMode {
    private MecanumDrive drive;
    private final Outtake outtake = new Outtake();
    private final Lift lift = new Lift();
    private final LightSystem light = new LightSystem();

    private Action toRung;
    private Action toSpike1;
    private Action toBasket1;
    private Action toSpike2;
    private Action toBasket2;
    private Action toAscent;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(12, 63.5, Math.toRadians(270)));
        outtake.init(hardwareMap);
        lift.init(hardwareMap);
        light.init(hardwareMap);

        Actions.runBlocking(
                new ParallelAction(
                         outtake.closeGrip()
                )
        );

        toRung = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-6, -31), Math.toRadians(-90.00))
                .build();

        toSpike1 = drive.actionBuilder(new Pose2d(-6, -31, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-49, -42))
                .build();

        toBasket1 = drive.actionBuilder(new Pose2d(-49,-42, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-55, -57), Math.toRadians(45.00))
                .build();

        toSpike2 = drive.actionBuilder(new Pose2d(-55, -57, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-60, -42), Math.toRadians(-90.00))
                .build();

        toBasket2 = drive.actionBuilder(new Pose2d(-60, -42, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-55, -57), Math.toRadians(45.00))
                .build();

        toAscent = drive.actionBuilder(new Pose2d(-55,-57, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-22, -11, Math.toRadians(180.00)), Math.toRadians(0.00))
                .build();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addData("Pos (XYTheta)", "%4.2f, %4.2f, %4.1f", drive.pose.position.x, drive.pose.position.y, drive.pose.position);
    }
}
