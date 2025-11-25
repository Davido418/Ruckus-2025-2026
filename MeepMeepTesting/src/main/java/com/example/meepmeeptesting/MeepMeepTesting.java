package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);
        int side = 1;



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(72, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(64, side*13, side*Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(40, side*18, side*Math.toRadians(90)), side*Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(37, side*28, side*Math.toRadians(90)), side*Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(35, side*61, side*Math.toRadians(90)), side*Math.toRadians(100))
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(18, side*22), side*Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-15, side*20, side*Math.toRadians(140)), side*Math.toRadians(180))

                .setTangent(0)
                .splineToSplineHeading(new Pose2d(0, side*20, side*Math.toRadians(90)), side*Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(12, side*35, side*Math.toRadians(90)), side*Math.toRadians(90))
                //.splineToSplineHeading(new Pose2d(12, 61, Math.toRadians(90)), Math.toRadians(90));
                .splineToConstantHeading(new Vector2d(12, side*44),side* Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(4, side*59), side*Math.toRadians(90))
                .setTangent(270)
                .splineToConstantHeading(new Vector2d(0, side*25), side*Math.toRadians(220))
                //.splineToSplineHeading(new Pose2d(0, 22, Math.toRadians(120)), Math.toRadians(220))
                .splineToSplineHeading(new Pose2d(-15, side*20, side*Math.toRadians(140)), side*Math.toRadians(200))
                .setTangent(70)
                .splineToSplineHeading(new Pose2d(-11.2, side*26, side*Math.toRadians(90)), side*Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-11.2, side*55, side*Math.toRadians(90)), side*Math.toRadians(90))
                .setTangent(180)
                .splineToSplineHeading(new Pose2d(-15, side*20, side*Math.toRadians(140)), side*Math.toRadians(280))
        //.splineToLinearHeading(new Pose2d(-15, 20, Math.toRadians(135)), Math.toRadians(270))
        //.splineToLinearHeading(new Pose2d(-15, 20, Math.toRadians(135)), Math.toRadians(270))


                /*.setTangent(0)

               /*.lineToX(36.5)

                .lineToY(55)
               .turn(Math.toRadians(90))
               .lineToX(0)
               .turn(Math.toRadians(90))
               .lineToY(0)
               .turn(Math.toRadians(90))*/
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}