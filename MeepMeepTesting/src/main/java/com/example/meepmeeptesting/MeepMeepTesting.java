package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(300);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive  -> {
                    Pose2d startPose = new Pose2d(25, 65, Math.toRadians(270));
                    Pose2d startPoseRed = new Pose2d(10, -50, Math.toRadians(90));

                    /*return drive.trajectorySequenceBuilder(startPose)
                            //go to bar
                            .splineTo(new Vector2d(0, 38), Math.toRadians(270))

                            .lineTo(new Vector2d(0, 34))

                            .lineTo(new Vector2d(0, 40))
                            //go to sample
                            .lineToSplineHeading(new Pose2d(-48, 37, Math.toRadians(270)))

                            //go to bucket
                            .lineToSplineHeading(new Pose2d(53, 53, Math.toRadians(225)))

                            //go to sample
                            .lineToSplineHeading(new Pose2d(-58, 37, Math.toRadians(270)))

                            //go to bucket
                            .lineToSplineHeading(new Pose2d(53, 53, Math.toRadians(225)))

                            //go to sample
                            .lineToSplineHeading(new Pose2d(-58, 25, Math.toRadians(180)))

                            //go to bucket
                            .lineToSplineHeading(new Pose2d(53, 53, Math.toRadians(225)))

                            .lineToSplineHeading(new Pose2d(-60, 58, Math.toRadians(180)))

                            .build();*/
                   return drive.trajectorySequenceBuilder(startPoseRed)
                           .splineTo(new Vector2d(0, -38), Math.toRadians(90))

                           .lineTo(new Vector2d(0, -34))

                           .lineTo(new Vector2d(0, -52))
                           //go to sample
                           .lineToSplineHeading(new Pose2d(48, -37, Math.toRadians(90)))

                           //go to bucket
                           .lineToSplineHeading(new Pose2d(-53, -53, Math.toRadians(45)))

                           //go to sample
                           .lineToSplineHeading(new Pose2d(58, -37, Math.toRadians(90)))

                           //go to bucket
                           .lineToSplineHeading(new Pose2d(-53, -53, Math.toRadians(45)))

                           //go to sample
                           .lineToSplineHeading(new Pose2d(58, -25, Math.toRadians(0)))

                           //go to bucket
                           .lineToSplineHeading(new Pose2d(-53, -53, Math.toRadians(45)))

                           .lineToSplineHeading(new Pose2d(60, -58, Math.toRadians(0)))


                            .build();
                });


        meepMeep.setBackground(
                ImageIO.read(new File("/home/siddhant/programming/robotics/into-the-sea/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/meepmeep.png"))
        ).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(myBot).start();
    }
}