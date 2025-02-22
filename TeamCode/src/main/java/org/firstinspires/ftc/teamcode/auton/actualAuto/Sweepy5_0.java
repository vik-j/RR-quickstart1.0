package org.firstinspires.ftc.teamcode.auton.actualAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auton.qolActions.qol;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@Autonomous(name = "Sweepy5_0", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class Sweepy5_0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-15, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Robot bot = new Robot(hardwareMap);
        qol q = new qol(bot);

        drive.enableHeadingCorrection();
        drive.enableTranslationalCorrection(1.0);
        drive.setCorrectionTimeout(0.5);

        bot.grippyClose();
        bot.flippy.setPosition(1);
        bot.twisty.setPosition(0);
        bot.resetEncoders();
        bot.sweepyUp();


        waitForStart();

        Robot.stopPid = false;



        //TODO: 13.38 V

        Action driveAction = drive.actionBuilder(beginPose)
                //TODO: 1st speci

                .afterTime(0.01, q.firstSpeci())
                .strafeToConstantHeading(new Vector2d(-8, 33.7))
                .afterTime(0, q.firstSpeci2())
                .waitSeconds(0.2)
                .afterTime(0, q.grippyOpen())
                .afterTime(0.1, q.combine(q.reset(), q.flippy(1)))
                .afterTime(1.25, q.autoSampleSweeping())
                .afterTime(1.5, q.sweepyDown())
                .splineToSplineHeading(new Pose2d(-36.61,47.9, Math.toRadians(-106.99)), Math.toRadians(180))
                .afterTime(0, q.flippy(0.4))

                //TODO: pickup 1st sample
                // .splineToSplineHeading(new Pose2d(-28.07,39.92, Math.toRadians(-145.5)), Math.toRadians(180))

                //TODO: drop off first sample
//                .strafeToSplineHeading(new Vector2d(-28.19, 42.58), Math.toRadians(128.5))
                .turnTo(Math.toRadians(147.6), new TurnConstraints(20, -20, 20))
                .afterTime(0, q.flippy(0.65))
                .splineToSplineHeading(new Pose2d(-45, 45.39, Math.toRadians(-113.26)), Math.toRadians(270))
                .afterTime(0, q.flippy(0.4))

                //TODO: pick up 2nd sample
                //.splineToSplineHeading(new Pose2d(-39.15, 40.48, Math.toRadians(-146.44)), Math.toRadians(270))

                //TODO: drop off 2nd sample
//                .strafeToSplineHeading(new Vector2d(-37.85, 42.6), Math.toRadians(130.4))
                .turnTo(Math.toRadians(139.3), new TurnConstraints(20, -20, 20))
                .afterTime(0, q.flippy(0.6))
                .splineToSplineHeading(new Pose2d(-51, 35.01, Math.toRadians(-137.92)), Math.toRadians(270))
                .afterTime(0, q.flippy(0.42))

                //TODO: pick up 3rd sample
                // .splineToSplineHeading(new Pose2d(-45.97, 39.097, Math.toRadians(-151.7)), Math.toRadians(270))

                //TODO: drop off 3rd sample
                .strafeToSplineHeading(new Vector2d(-41.11, 46.26), Math.toRadians(138.31))
                .afterTime(0, q.combine(q.flippy(0.6), q.sweepyUp()))


                .afterTime(0.75, q.specimenPickup())
                .waitSeconds(0.3)
                //TODO: pickup 2nd speci
                .strafeToLinearHeading(new Vector2d(-35.52, 47), Math.toRadians(-90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-35.52, 54), Math.toRadians(-90))
                .afterTime(0.25, q.grippyClose())
                .afterTime(0.7, q.flippy(0.8))
                .waitSeconds(0.15)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.3, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.1, q.specimenDeposit())
                //TODO: score 2nd speci
                .strafeToConstantHeading(new Vector2d(-3, 34.4))

                .afterTime(0.2, q.specimenDeposit2())
                .afterTime(0.55, q.grippyOpen())
                .waitSeconds(0.05)


                .afterTime(0.75, q.specimenPickup())
                //TODO: pickup 3rd speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53.5), Math.toRadians(-90))
                .afterTime(0, q.grippyClose())
                .afterTime(0.6, q.flippy(0.8))
                .waitSeconds(0)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.35, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.2, q.specimenDeposit())

                //TODO: score 3rd speci
                .strafeToConstantHeading(new Vector2d(0, 34.14))

                .afterTime(0.2, q.specimenDeposit2())
                .afterTime(0.45, q.grippyOpen())
                .waitSeconds(0.15)


                .afterTime(0.75, q.specimenPickup())

                //TODO: pick up 4th speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53), Math.toRadians(-90))
                .afterTime(0, q.grippyClose())
                .afterTime(0.6, q.flippy(0.7))
                .waitSeconds(0)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.4, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.37, q.specimenDeposit())
                //TODO: drop off 4th speci
                .strafeToConstantHeading(new Vector2d(5, 33.9))
                .afterTime(0.3, q.specimenDeposit2())
                .afterTime(0.65, q.grippyOpen())
                .waitSeconds(0.05)


                .afterTime(0.75, q.specimenPickup())


                //TODO: pick up 5th speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53.3), Math.toRadians(-90))
                .afterTime(0, q.grippyClose())
                .afterTime(0.6, q.flippy(0.7))
                .waitSeconds(0)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.4, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.32, q.specimenDeposit())
                //TODO: drop off 5th speci
                .strafeToConstantHeading(new Vector2d(7, 32.7))
                .afterTime(0.15, q.specimenDeposit2())
                .afterTime(0.47 , q.grippyOpen())
                .waitSeconds(0.2)
                .afterTime(2, q.reset())

                //TODO: park
                .strafeToConstantHeading(new Vector2d(-55, 55), new TranslationalVelConstraint(120), new ProfileAccelConstraint(-120, 120))

//                .waitSeconds(0.3)
//                .strafeTo(new Vector2d(0, 49))
//                .waitSeconds(0.5)
//                .strafeTo(new Vector2d(0, 36.5), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
//                .waitSeconds(1.5)
//                .strafeTo(new Vector2d(-36, 49), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
//                .waitSeconds(0.5)
//                .strafeToConstantHeading(new Vector2d(0, 36), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80,80))

                .build();

        Actions.runBlocking(
                new ParallelAction(
                        driveAction, bot.getPIDAction()
                )
        );
    }
}