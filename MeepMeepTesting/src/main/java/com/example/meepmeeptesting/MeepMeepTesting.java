package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .setDimensions(14, 13)
                .followTrajectorySequence(drive ->  drive.trajectorySequenceBuilder(new Pose2d(-36, -65, Math.toRadians(90.00)))
                        .lineToLinearHeading(new Pose2d(-6, -31, Math.toRadians(-90.00)))
                        .waitSeconds(4)
                        .lineTo(new Vector2d(-49, -42))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(-55, -57, Math.toRadians(45.00)))
                        .waitSeconds(6)
                        .lineToLinearHeading(new Pose2d(-60, -42, Math.toRadians(-90.00)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(-55, -57, Math.toRadians(45.00)))
                        .waitSeconds(6)
                        .splineToLinearHeading(new Pose2d(-22, -11, Math.toRadians(180.00)), Math.toRadians(0.00))
                        .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}