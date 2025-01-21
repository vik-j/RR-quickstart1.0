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

        bot.grippyClose();
        bot.flippy.setPosition(1);
        bot.twisty.setPosition(0);

        waitForStart();

        Robot.stopPid = false;

        Action armAction = drive.actionBuilder(beginPose)
                .afterTime(0.5, telemetryPacket -> {
                    bot.sampleScore();
                    return false;
                })
                .afterTime(1, telemetryPacket -> {
                    bot.sampleScore2();
                    return false;
                })
                .afterTime(1.2, telemetryPacket -> {
                    bot.flippy.setPosition(0.87);
                    return false;
                })
                .afterTime(2.4, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(2.5, telemetryPacket -> {
                    bot.reset();
                    return false;
                })
                .afterTime(3.5, telemetryPacket -> {
                    bot.samplePickup();
                    return false;
                })
                .afterTime(4.5, telemetryPacket -> {
                    bot.setPidValues(0, 1830);
                    return false;
                })
                .afterTime(5, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(5.2, telemetryPacket -> {
                    bot.setPidValues(0,0);
                    return false;
                })
                .afterTime(5.3, telemetryPacket -> {
                    bot.flippy.setPosition(1);
                    return false;
                })
                .afterTime(5.7, telemetryPacket -> {
                    bot.sampleScore();
                    return false;
                })
                .afterTime(6.6, telemetryPacket -> {
                    bot.sampleScore2();
                    bot.flippy.setPosition(0.87);
                    return false;
                })
                .afterTime(8, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .build();

        //TODO: 13.38 V

        Action driveAction = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(53,57), Math.toRadians(225))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(50, 50), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(53.5,55), Math.toRadians(225))
                .build();

        Actions.runBlocking(
                new ParallelAction(
                        driveAction, armAction, bot.getPIDAction()
                )
        );
    }
}



