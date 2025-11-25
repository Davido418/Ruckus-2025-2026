package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, 20, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(38, 30, Math.toRadians(90)), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(35, 55, Math.toRadians(90)), Math.toRadians(100))
                //1 Outtake
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(20, 25), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-15, 20, Math.toRadians(140)), Math.toRadians(150))
                //2Intake
                .setTangent(70)
                .splineToSplineHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(12, 55, Math.toRadians(90)), Math.toRadians(90))
                //2 Outtake
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(5, 25), Math.toRadians(225))
                .splineToSplineHeading(new Pose2d(-15, 20, Math.toRadians(140)), Math.toRadians(225))
                //3 Intake
                .setTangent(70)
                .splineToSplineHeading(new Pose2d(-12, 26, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-12, 55, Math.toRadians(90)), Math.toRadians(90))
                //3 Outtake
                .setTangent(180)
                .splineToSplineHeading(new Pose2d(-15, 20, Math.toRadians(140)), Math.toRadians(270))
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