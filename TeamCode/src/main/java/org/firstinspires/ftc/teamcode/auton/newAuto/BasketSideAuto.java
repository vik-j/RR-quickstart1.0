package org.firstinspires.ftc.teamcode.auton.newAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ejml.equation.Operation;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@Autonomous(name = "BasketSideAuto", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class BasketSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(9, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Robot bot = new Robot(hardwareMap);
        bot.resetEncoders();
        drive.enableHeadingCorrection();
        drive.disableTranslationalCorrection();

        bot.grippyClose();
        bot.flippy.setPosition(1);
        bot.twisty.setPosition(0);

        double optimizationOffset = 1+0.5;

        waitForStart();

        Robot.stopPid = false;

        Action armAction = drive.actionBuilder(beginPose)
                .afterTime(0.01, telemetryPacket -> {
                    bot.newSpeci();
                    return false;
                })
                .afterTime(1.4, telemetryPacket -> {
                    bot.newSpeci2();
                    return false;
                })
                .afterTime(1.8, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(3.5, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(4.25, telemetryPacket -> {
                    bot.twisty.setPosition(0.33);
                    bot.sampleUp();
                    return false;
                })
                .afterTime(5, telemetryPacket -> {
                    bot.setPidValues(425, 2300);
                    return false;
                })
                .afterTime(6.5, telemetryPacket -> {
                    bot.setPivotMultiplier(0.001);
                    bot.setPidValues(0, 2300);
                    return false;
                })
                .afterTime(7.5, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(8.5, telemetryPacket -> {
                    bot.setPivotMultiplier(1);
                    bot.sampleDeposit();
                    return false;
                })
                .afterTime(10, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(10.5, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(10.75, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(11.5, telemetryPacket -> {
                    bot.sampleUp();
                    return false;
                })
                .afterTime(12.5, telemetryPacket -> {
                    bot.setPidValues(425, 2300);
                    return false;
                })
                .afterTime(14, telemetryPacket -> {
                    bot.setPivotMultiplier(0.001);
                    bot.setPidValues(0, 2300);
                    return false;
                })
                .afterTime(15, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(16, telemetryPacket -> {
                    bot.setPivotMultiplier(1);
                    bot.sampleDeposit();
                    return false;
                })
                .afterTime(17.5, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(18, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(18.25, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(19, telemetryPacket -> {
                    bot.twisty.setPosition(0.86);
                    bot.sampleUp();
                    return false;
                })
                .afterTime(20, telemetryPacket -> {
                    bot.setPidValues(425, 2300);
                    return false;
                })
                .afterTime(21.5, telemetryPacket -> {
                    bot.setPivotMultiplier(0.001);
                    bot.setPidValues(0, 2300);
                    return false;
                })
                .afterTime(22.5, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(23.5, telemetryPacket -> {
                    bot.setPivotMultiplier(1);
                    bot.sampleDeposit();
                    return false;
                })
                .afterTime(25, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(25.5, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(25.75, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(26.5, telemetryPacket -> {
                    bot.flippy.setPosition(1);
                    bot.setPidValues(1100, 2000);
                    return false;
                })
                .build();

        //TODO: 13.38 V

        Action driveAction = drive.actionBuilder(beginPose)
                .splineToSplineHeading(new Pose2d(-6,44.2, Math.toRadians(270)), Math.toRadians(180))
                .waitSeconds(0.2)
                .strafeToConstantHeading(new Vector2d(52.5, 55.5), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(54.634, 51, Math.toRadians(-98.687)), Math.toRadians(270), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 60))
                .waitSeconds(3.5)
                .strafeToLinearHeading(new Vector2d(52.5, 55.5), Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(58.35, 51.285, Math.toRadians(-87)), Math.toRadians(270), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(3.5)
                .strafeToLinearHeading(new Vector2d(52.5, 55.5), Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(58.185, 48.67, Math.toRadians(-60.88)), Math.toRadians(270), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(3.5)
                .strafeToLinearHeading(new Vector2d(52.5, 55.5), Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(36, 20, Math.toRadians(180)), Math.toRadians((180)))
                .splineToLinearHeading(new Pose2d(22, 20, Math.toRadians(180)), Math.toRadians((180)))

                .build();

        Actions.runBlocking(
                new ParallelAction(
                        driveAction, armAction, bot.getPIDAction()
                )
        );
    }
}



