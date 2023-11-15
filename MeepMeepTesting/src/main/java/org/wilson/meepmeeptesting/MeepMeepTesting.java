package org.wilson.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep;
        meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17.2, 16.75)
                .setDriveTrainType(DriveTrainType.MECANUM)
                // Set bot constraints to match 2023 bot
                .setConstraints(35, 25, Math.toRadians(150), Math.toRadians(100), 15.73)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39, -62.125, Math.toRadians(90)))
                                .waitSeconds(2)
                                .lineToSplineHeading(new Pose2d(-35.5 -11.8, -23.5, Math.toRadians(0)) )
                                // .forward(25)
                                .waitSeconds(.5)
                                .forward(1)
                                .waitSeconds(.5)
                                .splineToConstantHeading( new Vector2d(-35.5, -10.5), Math.toRadians(0))
                               // .lineToSplineHeading(new Pose2d(-39, -60, Math.toRadians(0)) )
                                .lineTo( new Vector2d(12, -10.5))
                                .lineToConstantHeading( new Vector2d(48, -36) )
                                .build()
                                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}