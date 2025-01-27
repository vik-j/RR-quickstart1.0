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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@Autonomous(name = "SpeciSideAutoNEW", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class SpeciSideAutoNEW extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-15, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Robot bot = new Robot(hardwareMap);

        drive.enableHeadingCorrection();
        drive.enableTranslationalCorrection(1.0);

        bot.grippyClose();
        bot.flippy.setPosition(1);
        bot.twisty.setPosition(0);
        bot.resetEncoders();

        waitForStart();

        Robot.stopPid = false;



        //TODO: 13.38 V

        Action driveAction = drive.actionBuilder(beginPose)
                .afterTime(0.01, telemetryPacket -> {
                    bot.newSpeci();
                    return false;
                })
                .strafeToConstantHeading(new Vector2d(-8.5, 34.5))
                .afterTime(0, telemetryPacket -> {
                    bot.newSpeci2();
                    return false;
                })
                .waitSeconds(0.5)
                .afterTime(0, telemetryPacket -> {
                    bot.reset();
                    bot.flippy.setPosition(1);
                    return false;
                })
                .afterTime(1.25, telemetryPacket -> {
                    bot.autoSamplePickup();
                    return false;
                })
                .splineToSplineHeading(new Pose2d(-28.07,39.92, Math.toRadians(-145.5)), Math.toRadians(180))
                .afterTime(0.5, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.75, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(0.9, telemetryPacket -> {
                    bot.flippy.setPosition(0.5);
                    return false;
                })
                .waitSeconds(0.8)
                .turnTo(Math.toRadians(145), new TurnConstraints(60, -60, 60))
                .afterTime(0, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .splineToSplineHeading(new Pose2d(-39.15, 40.48, Math.toRadians(-146.44)), Math.toRadians(270))
                .afterTime(0.5, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.75, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(0.9, telemetryPacket -> {
                    bot.flippy.setPosition(0.5);
                    return false;
                })
                .waitSeconds(0.8)
                .turnTo(Math.toRadians(121.95), new TurnConstraints(80, -80, 80))
                .afterTime(0, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .splineToSplineHeading(new Pose2d(-47.47, 39.097, Math.toRadians(-151.7)), Math.toRadians(270))
                .afterTime(0.5, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.75, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(0.9, telemetryPacket -> {
                    bot.flippy.setPosition(0.5);
                    bot.twisty.setPosition(0);
                    return false;
                })
                .waitSeconds(0.8)
                .splineToSplineHeading(new Pose2d(-43.2, 47.193, Math.toRadians(145.3)), Math.toRadians(90), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80,80))
                .afterTime(0, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.specimenPickup();
                    return false;
                })
                .splineToSplineHeading(new Pose2d(-35.52, 52.64, Math.toRadians(-90.5)), Math.toRadians(90))

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



