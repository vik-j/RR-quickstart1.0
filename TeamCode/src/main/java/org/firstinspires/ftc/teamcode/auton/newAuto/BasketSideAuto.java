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
import org.firstinspires.ftc.teamcode.BasketSideMecanumDrive;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@Autonomous(name = "BasketSideAuto", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class BasketSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(40, 64, Math.toRadians(180));
        BasketSideMecanumDrive drive = new BasketSideMecanumDrive(hardwareMap, beginPose);
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
//                .afterTime(0.5, telemetryPacket -> {
//                    bot.sampleScore();
//                    return false;
//                })
//                .afterTime(1.2, telemetryPacket -> {
//                    bot.flippy.setPosition(0.6);
//                    return false;
//                })
                .afterTime(0.01, telemetryPacket -> {
                    bot.sampleDeposit();
                    return false;
                })
                .afterTime(2.5, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(3, telemetryPacket -> {
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

//                .afterTime(2.5, telemetryPacket -> {
//                    bot.sampleScore3();
//                    bot.flippy.setPosition(0.87);
//                    return false;
//                })
//                .afterTime(3.1, telemetryPacket -> {
//                    bot.grippyOpen();
//                    return false;
//                })
//                .afterTime(3.5, telemetryPacket -> {
//                    bot.reset();
//                    return false;
//                })
//
//
//
//                .afterTime(6, telemetryPacket -> {
//                    bot.samplePickup();
//                    return false;
//                })
//                .afterTime(7, telemetryPacket -> {
//                    bot.setPidValues(0, 1730);
//                    return false;
//                })
//                .afterTime(7.25, telemetryPacket -> {
//                    bot.grippyClose();
//                    return false;
//                })
//                .afterTime(8.9 - optimizationOffset, telemetryPacket -> {
//                    bot.setPidValues(0,0);
//                    return false;
//                })
//                .afterTime(9.1 - optimizationOffset, telemetryPacket -> {
//                    bot.flippy.setPosition(1);
//                    return false;
//                })
//                .afterTime(10.25 - optimizationOffset, telemetryPacket -> {
//                    bot.sampleScore();
//                    bot.flippy.setPosition(0.6);
//                    return false;
//                })
//                .afterTime(11.25 - optimizationOffset, telemetryPacket -> {
//                    bot.sampleScore2();
//                    return false;
//                })
//                .afterTime(12.35 - optimizationOffset, telemetryPacket -> {
//                    bot.sampleScore3();
//                    bot.flippy.setPosition(0.87);
//                    return false;
//                })
//                .afterTime(12.75 - optimizationOffset, telemetryPacket -> {
//                    bot.grippyOpen();
//                    return false;
//                })
//                .afterTime(12.85 - optimizationOffset, telemetryPacket -> {
//                    bot.flippy.setPosition(0.4);
//                    return false;
//                })
//                .afterTime(13.15 - optimizationOffset, telemetryPacket -> {
//                    bot.reset();
//                    return false;
//                })
//
//
//                .afterTime(14.5 - optimizationOffset, telemetryPacket -> {
//                    bot.samplePickup();
//                    return false;
//                })
//                .afterTime(15 - optimizationOffset, telemetryPacket -> {
//                    bot.setPidValues(0, 1730);
//                    return false;
//                })
//                .afterTime(15.2 - optimizationOffset, telemetryPacket -> {
//                    bot.grippyClose();
//                    return false;
//                })
//                .afterTime(15.5 - optimizationOffset, telemetryPacket -> {
//                    bot.setPidValues(0,0);
//                    return false;
//                })
//                .afterTime(15.75 - optimizationOffset, telemetryPacket -> {
//                    bot.flippy.setPosition(1);
//                    return false;
//                })
//                .afterTime(16.5 - optimizationOffset, telemetryPacket -> {
//                    bot.sampleScore();
//                    bot.flippy.setPosition(0.6);
//                    return false;
//                })
//                .afterTime(17.25 - optimizationOffset, telemetryPacket -> {
//                    bot.sampleScore2();
//                    return false;
//                })
//                .afterTime(18.5 - optimizationOffset, telemetryPacket -> {
//                    bot.sampleScore3();
//                    bot.flippy.setPosition(0.87);
//                    return false;
//                })
//                .afterTime(18.9 - optimizationOffset, telemetryPacket -> {
//                    bot.grippyOpen();
//                    return false;
//                })
//                .afterTime(19.1 - optimizationOffset, telemetryPacket -> {
//                    bot.flippy.setPosition(0.4);
//                    return false;
//                })
//                .afterTime(19.25 - optimizationOffset, telemetryPacket -> {
//                    bot.reset();
//                    return false;
//                })
//
//
//                .afterTime(21 - optimizationOffset, telemetryPacket -> {
//                    bot.samplePickup();
//                    return false;
//                })
//                .afterTime(22 - optimizationOffset, telemetryPacket -> {
//                    bot.setPidValues(0, 1730);
//                    return false;
//                })
//                .afterTime(22.25 - optimizationOffset, telemetryPacket -> {
//                    bot.grippyClose();
//                    return false;
//                })
//                .afterTime(22.5 - optimizationOffset, telemetryPacket -> {
//                    bot.setPidValues(0,0);
//                    return false;
//                })
//                .afterTime(22.75 - optimizationOffset, telemetryPacket -> {
//                    bot.flippy.setPosition(1);
//                    return false;
//                })
//                .afterTime(24.25 - optimizationOffset, telemetryPacket -> {
//                    bot.sampleScore();
//                    bot.flippy.setPosition(0.6);
//                    return false;
//                })
//                .afterTime(25.25 - optimizationOffset, telemetryPacket -> {
//                    bot.sampleScore2();
//                    return false;
//                })
//                .afterTime(26.5 - optimizationOffset, telemetryPacket -> {
//                    bot.sampleScore3();
//                    bot.flippy.setPosition(0.87);
//                    return false;
//                })
//                .afterTime(26.9 - optimizationOffset, telemetryPacket -> {
//                    bot.grippyOpen();
//                    return false;
//                })
//                .afterTime(27 - optimizationOffset, telemetryPacket -> {
//                    bot.flippy.setPosition(0.4);
//                    return false;
//                })
//                .afterTime(27.25 - optimizationOffset, telemetryPacket -> {
//                    bot.reset();
//                    return false;
//                })
//                .afterTime(28.25 - optimizationOffset, telemetryPacket -> {
//                    bot.setPidValues(1100, 2000);
//                    bot.flippy.setPosition(0.5);
//                    return false;
//                })
                .build();

        //TODO: 13.38 V

        Action driveAction = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(48,48), Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(0.5)
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
                .splineToLinearHeading(new Pose2d(18, 20, Math.toRadians(180)), Math.toRadians((180)))
//                .strafeToLinearHeading(new Vector2d(52, 55), Math.toRadians(225))
//                .waitSeconds(4)
//                .strafeToLinearHeading(new Vector2d(49.5, 48.25), Math.toRadians(270), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 60))
//                .waitSeconds(5 - optimizationOffset)
//                .strafeToLinearHeading(new Vector2d(53.5,52), Math.toRadians(225), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60,60))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(60, 48.25), Math.toRadians(270), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-50, 50))
//                .waitSeconds(3)
//                .strafeToLinearHeading(new Vector2d(52,55), Math.toRadians(225), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60,60))
//                .waitSeconds(2.4)
//                .strafeToLinearHeading(new Vector2d(61, 48), Math.toRadians(292.5), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60,60))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(53.5,53), Math.toRadians(225), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60,60))
//                .waitSeconds(3.5)
//                .splineToLinearHeading(new Pose2d(36, 10, Math.toRadians(180)), Math.toRadians((180)))
//                .splineToLinearHeading(new Pose2d(22, 10, Math.toRadians(180)), Math.toRadians((180)))
                .build();

        Actions.runBlocking(
                new ParallelAction(
                        driveAction, armAction, bot.getPIDAction()
                )
        );
    }
}



