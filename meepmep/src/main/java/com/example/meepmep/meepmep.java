package com.example.meepmep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d beginPose1 = new Pose2d(39, 63.25, Math.toRadians(180));
        Pose2d reflectedPose1 = new Pose2d(-39, -63.25, Math.toRadians(0));
        Pose2d beginPose2 = new Pose2d(-15, 61.5, Math.toRadians(270));
        Pose2d reflectedPose2 = new Pose2d(15, -61.5, Math.toRadians(90));

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        RoadRunnerBotEntity bot3 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        RoadRunnerBotEntity bot4 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        bot1.runAction(bot1.getDrive().actionBuilder(beginPose1)
                .strafeToLinearHeading(new Vector2d(48,50), Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(54, 57.5), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(53.734, 51.5, Math.toRadians(-98.687)), Math.toRadians(270), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 60))
                .waitSeconds(2.2)
                .strafeToLinearHeading(new Vector2d(54, 57), Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(58.35, 52.285, Math.toRadians(-87)), Math.toRadians(270), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(2.3)
                .strafeToLinearHeading(new Vector2d(54, 57), Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(59.185, 48.5, Math.toRadians(-60.88)), Math.toRadians(270), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(55.5, 57.5), Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(36, 22.5, Math.toRadians(180)), Math.toRadians((180)))
                .splineToLinearHeading(new Pose2d(15, 22.5, Math.toRadians(180)), Math.toRadians((180)))
                .build());

        bot2.runAction(bot2.getDrive().actionBuilder(reflectedPose1)
                .strafeToLinearHeading(new Vector2d(-48, -50), Math.toRadians(-45))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-54, -57.5))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-53.734, -51.5, Math.toRadians(81.313)), Math.toRadians(90))
                .waitSeconds(2.2)
                .strafeToLinearHeading(new Vector2d(-54, -57), Math.toRadians(-45))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-58.35, -52.285, Math.toRadians(93)), Math.toRadians(90))
                .waitSeconds(2.3)
                .strafeToLinearHeading(new Vector2d(-54, -57), Math.toRadians(-45))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-59.185, -48.5, Math.toRadians(119.12)), Math.toRadians(90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-55.5, -57.5), Math.toRadians(-45))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-36, -22.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-15, -22.5, Math.toRadians(0)), Math.toRadians(0))
                .build());

        bot3.runAction(bot3.getDrive().actionBuilder(beginPose2)
                .strafeToConstantHeading(new Vector2d(-8, 33.7))
                .splineToSplineHeading(new Pose2d(-28.89, 39.58, Math.toRadians(-140.26)), Math.toRadians(180))
                .turnTo(Math.toRadians(128.5))
                .splineToSplineHeading(new Pose2d(-38.45, 39.6, Math.toRadians(-140.91)), Math.toRadians(270))
                .turnTo(Math.toRadians(130.4))
                .splineToSplineHeading(new Pose2d(-47.1, 35.5, Math.toRadians(-156.46)), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(-44.2, 41.83), Math.toRadians(132))
                .strafeToLinearHeading(new Vector2d(-35.52, 47), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-35.52, 54), Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-4, 34.4))
                .strafeToConstantHeading(new Vector2d(0, 34.14))
                .strafeToConstantHeading(new Vector2d(7, 33.9))
                .strafeToConstantHeading(new Vector2d(8, 33.6))
                .strafeToConstantHeading(new Vector2d(-55, 55))
                .build());

        bot4.runAction(bot4.getDrive().actionBuilder(reflectedPose2)
                .strafeToConstantHeading(new Vector2d(8, -33.7))
                .splineToSplineHeading(new Pose2d(28.89, -39.58, Math.toRadians(40.26)), Math.toRadians(0))
                .turnTo(Math.toRadians(-128.5))
                .splineToSplineHeading(new Pose2d(38.45, -39.6, Math.toRadians(40.91)), Math.toRadians(90))
                .turnTo(Math.toRadians(-130.4))
                .splineToSplineHeading(new Pose2d(47.1, -35.5, Math.toRadians(23.54)), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(44.2, -41.83), Math.toRadians(-48))
                .strafeToLinearHeading(new Vector2d(35.52, -47), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(35.52, -54), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(4, -34.4))
                .strafeToConstantHeading(new Vector2d(0, -34.14))
                .strafeToConstantHeading(new Vector2d(-7, -33.9))
                .strafeToConstantHeading(new Vector2d(-8, -33.6))
                .strafeToConstantHeading(new Vector2d(55, -55))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot1)
                .addEntity(bot2)
                .addEntity(bot3)
                .addEntity(bot4)
                .start();
    }
}
