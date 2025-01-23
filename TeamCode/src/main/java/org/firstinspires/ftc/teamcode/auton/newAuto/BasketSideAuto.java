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
        Pose2d beginPose = new Pose2d(40, 64, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Robot bot = new Robot(hardwareMap);
        bot.resetArm();

        bot.grippyClose();
        bot.flippy.setPosition(1);
        bot.twisty.setPosition(0);

        double optimizationOffset = 1+0.5;

        waitForStart();

        Robot.stopPid = false;

        Action armAction = drive.actionBuilder(beginPose)
                .afterTime(0.5, telemetryPacket -> {
                    bot.sampleScore();
                    return false;
                })
                .afterTime(1.2, telemetryPacket -> {
                    bot.flippy.setPosition(0.6);
                    return false;
                })
                .afterTime(1.5, telemetryPacket -> {
                    bot.sampleScore2();
                    return false;
                })
                .afterTime(2.5, telemetryPacket -> {
                    bot.sampleScore3();
                    bot.flippy.setPosition(0.87);
                    return false;
                })
                .afterTime(3.1, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(3.5, telemetryPacket -> {
                    bot.reset();
                    return false;
                })



                .afterTime(6, telemetryPacket -> {
                    bot.samplePickup();
                    return false;
                })
                .afterTime(7, telemetryPacket -> {
                    bot.setPidValues(0, 1730);
                    return false;
                })
                .afterTime(7.25, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(8.9 - optimizationOffset, telemetryPacket -> {
                    bot.setPidValues(0,0);
                    return false;
                })
                .afterTime(9.1 - optimizationOffset, telemetryPacket -> {
                    bot.flippy.setPosition(1);
                    return false;
                })
                .afterTime(10.25 - optimizationOffset, telemetryPacket -> {
                    bot.sampleScore();
                    bot.flippy.setPosition(0.6);
                    return false;
                })
                .afterTime(11.25 - optimizationOffset, telemetryPacket -> {
                    bot.sampleScore2();
                    return false;
                })
                .afterTime(12.35 - optimizationOffset, telemetryPacket -> {
                    bot.sampleScore3();
                    bot.flippy.setPosition(0.87);
                    return false;
                })
                .afterTime(12.75 - optimizationOffset, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(12.95 - optimizationOffset, telemetryPacket -> {
                    bot.reset();
                    return false;
                })


                .afterTime(14.5 - optimizationOffset, telemetryPacket -> {
                    bot.samplePickup();
                    return false;
                })
                .afterTime(15 - optimizationOffset, telemetryPacket -> {
                    bot.setPidValues(0, 1730);
                    return false;
                })
                .afterTime(15.2 - optimizationOffset, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(15.5 - optimizationOffset, telemetryPacket -> {
                    bot.setPidValues(0,0);
                    return false;
                })
                .afterTime(15.75 - optimizationOffset, telemetryPacket -> {
                    bot.flippy.setPosition(1);
                    return false;
                })
                .afterTime(16.5 - optimizationOffset, telemetryPacket -> {
                    bot.sampleScore();
                    bot.flippy.setPosition(0.6);
                    return false;
                })
                .afterTime(17.25 - optimizationOffset, telemetryPacket -> {
                    bot.sampleScore2();
                    return false;
                })
                .afterTime(18.5 - optimizationOffset, telemetryPacket -> {
                    bot.sampleScore3();
                    bot.flippy.setPosition(0.87);
                    return false;
                })
                .afterTime(18.9 - optimizationOffset, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(19.25 - optimizationOffset, telemetryPacket -> {
                    bot.reset();
                    return false;
                })


                .afterTime(21 - optimizationOffset, telemetryPacket -> {
                    bot.samplePickup();
                    return false;
                })
                .afterTime(22 - optimizationOffset, telemetryPacket -> {
                    bot.setPidValues(0, 1730);
                    return false;
                })
                .afterTime(22.25 - optimizationOffset, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(22.5 - optimizationOffset, telemetryPacket -> {
                    bot.setPidValues(0,0);
                    return false;
                })
                .afterTime(22.75 - optimizationOffset, telemetryPacket -> {
                    bot.flippy.setPosition(1);
                    return false;
                })
                .afterTime(24.25 - optimizationOffset, telemetryPacket -> {
                    bot.sampleScore();
                    bot.flippy.setPosition(0.6);
                    return false;
                })
                .afterTime(25.25 - optimizationOffset, telemetryPacket -> {
                    bot.sampleScore2();
                    return false;
                })
                .afterTime(26.5 - optimizationOffset, telemetryPacket -> {
                    bot.sampleScore3();
                    bot.flippy.setPosition(0.87);
                    return false;
                })
                .afterTime(26.9 - optimizationOffset, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(27.25 - optimizationOffset, telemetryPacket -> {
                    bot.reset();
                    return false;
                })
                .afterTime(28.25 - optimizationOffset, telemetryPacket -> {
                    bot.setPidValues(1100, 2000);
                    bot.flippy.setPosition(0.5);
                    return false;
                })
                .build();

        //TODO: 13.38 V

        Action driveAction = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(52,55), Math.toRadians(225))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(49.5, 48.25), Math.toRadians(270), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-50, 50))
                .waitSeconds(5 - optimizationOffset)
                .strafeToLinearHeading(new Vector2d(53.5,52), Math.toRadians(225))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(60, 48.25), Math.toRadians(270), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-50, 50))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(52,55), Math.toRadians(225))
                .waitSeconds(2.4)
                .strafeToLinearHeading(new Vector2d(61, 48), Math.toRadians(292.5))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(53.5,53), Math.toRadians(225))
                .waitSeconds(3.5)
                .splineToLinearHeading(new Pose2d(36, 10, Math.toRadians(180)), Math.toRadians((180)))
                .splineToLinearHeading(new Pose2d(22, 10, Math.toRadians(180)), Math.toRadians((180)))
                .build();

        Actions.runBlocking(
                new ParallelAction(
                        driveAction, armAction, bot.getPIDAction()
                )
        );
    }
}



