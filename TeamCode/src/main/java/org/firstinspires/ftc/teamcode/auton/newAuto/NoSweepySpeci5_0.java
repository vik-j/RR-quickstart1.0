package org.firstinspires.ftc.teamcode.auton.newAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@Autonomous(name = "NoSweepySpeci5_0", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class NoSweepySpeci5_0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-15, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Robot bot = new Robot(hardwareMap);

        drive.enableHeadingCorrection();
        drive.enableTranslationalCorrection(1.0);
        drive.setCorrectionTimeout(1.0);

        bot.grippyClose();
        bot.flippy.setPosition(1);
        bot.twisty.setPosition(0);
        bot.resetEncoders();


        waitForStart();

        Robot.stopPid = false;



        //TODO: 13.38 V

        Action driveAction = drive.actionBuilder(beginPose)
                //TODO: 1st speci

                .afterTime(0.01, telemetryPacket -> {
                    bot.newSpeci();
                    return false;
                })
                .strafeToConstantHeading(new Vector2d(-8.5, 34.4))
                .afterTime(0, telemetryPacket -> {
                    bot.newSpeci2();
                    return false;
                })
                .waitSeconds(0.2)
                .afterTime(0, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(0.1, telemetryPacket -> {
                    bot.reset();
                    bot.flippy.setPosition(1);
                    return false;
                })
                .afterTime(1.25, telemetryPacket -> {
                    bot.autoSamplePickup();
                    return false;
                })
                .splineToSplineHeading(new Pose2d(-28,39.92, Math.toRadians(-145.5)), Math.toRadians(180))

                //TODO: pickup 1st sample
                // .splineToSplineHeading(new Pose2d(-28.07,39.92, Math.toRadians(-145.5)), Math.toRadians(180))
                .afterTime(0.4, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.65, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(0.9, telemetryPacket -> {
                    bot.flippy.setPosition(0.6);
                    return false;
                })
                .waitSeconds(0.7)

                //TODO: drop off first sample
                .turnTo(Math.toRadians(130), new TurnConstraints(20, -20, 20))
                .afterTime(0, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .splineToSplineHeading(new Pose2d(-38.6, 40.48, Math.toRadians(-146.44)), Math.toRadians(270))

                //TODO: pick up 2nd sample
                //.splineToSplineHeading(new Pose2d(-39.15, 40.48, Math.toRadians(-146.44)), Math.toRadians(270))
                .afterTime(0.6, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.8, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(1, telemetryPacket -> {
                    bot.flippy.setPosition(0.6);
                    return false;
                })

                //TODO: drop off 2nd sample
                .waitSeconds(0.4)
                .turnTo(Math.toRadians(121.95), new TurnConstraints(20, -20, 20))
                .afterTime(0, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .splineToSplineHeading(new Pose2d(-46.25, 39.5, Math.toRadians(-151.7)), Math.toRadians(270))

                //TODO: pick up 3rd sample
                // .splineToSplineHeading(new Pose2d(-45.97, 39.097, Math.toRadians(-151.7)), Math.toRadians(270))
                .afterTime(0.4, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(1.25, telemetryPacket -> {
                    bot.twisty.setPosition(0.5);
                    bot.flippy.setPosition(0.6);
                    return false;
                })
                .afterTime(1.3, telemetryPacket -> {
                    bot.reset();
                    bot.flippy.setPosition(0.6);
                    return false;
                })
                .waitSeconds(0.6)
                .afterTime(0.6, telemetryPacket -> {
                    bot.setPidValues(0, 800);
                    return false;
                })

                //TODO: drop off 3rd sample
                .strafeToSplineHeading(new Vector2d(-39.2, 48.193), Math.toRadians(120.3))
                .afterTime(0, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })


                .afterTime(0.75, telemetryPacket -> {
                    bot.specimenPickup();
                    return false;
                })
                .waitSeconds(0.3)
                //TODO: pickup 2nd speci
                .strafeToLinearHeading(new Vector2d(-35.52, 47), Math.toRadians(-90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-35.52, 54), Math.toRadians(-90))
                .afterTime(0.35, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.flippy.setPosition(0.8);
                    return false;
                })
                .waitSeconds(0.15)
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(0.3, telemetryPacket -> {
                    bot.speciScoreReset();
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(1, telemetryPacket -> {
                    bot.specimenDeposit();
                    return false;
                })
                //TODO: score 2nd speci
                .strafeToConstantHeading(new Vector2d(-4, 34.79))

                .afterTime(0.2, telemetryPacket -> {
                    bot.specimenDeposit2();
                    return false;
                })
                .afterTime(0.45, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .waitSeconds(0.05)


                .afterTime(0.75, telemetryPacket -> {
                    bot.specimenPickup();
                    return false;
                })
                //TODO: pickup 3rd speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53.5), Math.toRadians(-90))
                .afterTime(0, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(0.4, telemetryPacket -> {
                    bot.flippy.setPosition(0.8);
                    return false;
                })
                .waitSeconds(0)
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(0.35, telemetryPacket -> {
                    bot.speciScoreReset();
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(1.1, telemetryPacket -> {
                    bot.specimenDeposit();
                    return false;
                })

                //TODO: score 3rd speci
                .strafeToConstantHeading(new Vector2d(0, 34.54))

                .afterTime(0.2, telemetryPacket -> {
                    bot.specimenDeposit2();
                    return false;
                })
                .afterTime(0.45, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .waitSeconds(0.15)


                .afterTime(0.75, telemetryPacket -> {
                    bot.specimenPickup();
                    return false;
                })

                //TODO: pick up 4th speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53), Math.toRadians(-90))
                .afterTime(0, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(0.4, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .waitSeconds(0)
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(0.4, telemetryPacket -> {
                    bot.speciScoreReset();
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(1.22, telemetryPacket -> {
                    bot.specimenDeposit();
                    return false;
                })
                //TODO: drop off 4th speci
                .strafeToConstantHeading(new Vector2d(7, 34.1))
                .afterTime(0.3, telemetryPacket -> {
                    bot.specimenDeposit2();
                    return false;
                })
                .afterTime(0.65, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .waitSeconds(0.05)

                // TODO: COMMENT EVERYTHING OUT FROM UNDER HERE AND UNCOMMENT PARK FOR 4+0

                .afterTime(0.75, telemetryPacket -> {
                    bot.specimenPickup();
                    return false;
                })


                //TODO: pick up 5th speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53), Math.toRadians(-90))
                .afterTime(0, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(0.4, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .waitSeconds(0)
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(0.4, telemetryPacket -> {
                    bot.speciScoreReset();
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(1.32, telemetryPacket -> {
                    bot.specimenDeposit();
                    return false;
                })
                //TODO: drop off 5th speci
                .strafeToConstantHeading(new Vector2d(8, 34.44))
                .afterTime(0.15, telemetryPacket -> {
                    bot.specimenDeposit2();
                    return false;
                })
                .afterTime(0.4, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .waitSeconds(0.2)

                //TODO: park
                .strafeToConstantHeading(new Vector2d(-60, 60), new TranslationalVelConstraint(120), new ProfileAccelConstraint(-120, 120))

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