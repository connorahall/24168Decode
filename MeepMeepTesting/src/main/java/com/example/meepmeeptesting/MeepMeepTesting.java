package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.jetbrains.annotations.NotNull;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(79.26939782646593, 120, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-50, -50, Math.toRadians(50)))
//                        .lineToLinearHeading(new Pose2d(-20, -18, Math.toRadians(230)))
                    .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-50, 50, Math.toRadians(-40)))
                        .lineToLinearHeading(new Pose2d(-20, 18, Math.toRadians(140)))

//                        .splineToLinearHeading(new Pose2d(-12, -30, Math.toRadians(270)), Math.toRadians(270))
//                        .lineToLinearHeading(new Pose2d(-12, -50, Math.toRadians(270)))
//                        .setReversed(true)
//                        .splineToLinearHeading(new Pose2d(-16, -10, Math.toRadians(220)), Math.toRadians(90))
                        .build());


        Image img = null;
        try {
            img = ImageIO.read(new File("C:\\Users\\Dragon Bytes\\StudioProjects\\24168Decode\\decode-custom-field-images-meepmeep-compatible-printer-v0-xsjhmvxpoonf1.png"));
        }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}