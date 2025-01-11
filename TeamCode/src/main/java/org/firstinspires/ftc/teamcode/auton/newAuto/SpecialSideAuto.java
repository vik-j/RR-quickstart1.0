package org.firstinspires.ftc.teamcode.auton.newAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@Autonomous(name = "SpecialSideAuto", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class SpecialSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(15, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Robot bot = new Robot(hardwareMap);

        bot.grippyClose();
        bot.flippy.setPosition(0);
        bot.twisty.setPosition(1);
        Actions.runBlocking(bot.setPidVals(755, 0));
        Actions.runBlocking(bot.pidfLoopSingular(755));

        waitForStart();

        Robot.stopPid = false;

        Action armAction = drive.actionBuilder(beginPose)
                .afterTime(0.01, telemetryPacket -> {
                    bot.speciMacro();
                    return false;
                })
                .afterTime(2.5, telemetryPacket -> {
                    bot.grippyOpen();
                    bot.reset();
                    return false;
                })
                .afterTime(3, telemetryPacket -> {
                    bot.samplePickup();
                    return false;
                })
                .build();

        Action driveAction = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(6, -26), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(34.5515,-43.2), Math.toRadians(53.375))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(34.95, -43.635), Math.toRadians(-50.7))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(43.67, -41.8), Math.toRadians(51.777))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(45.02, -43.37), Math.toRadians(-45.44))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(52.93, -41.32), Math.toRadians(51.15))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(36.481, -46.63), Math.toRadians(-90))
                .build();

        Actions.runBlocking(
                new ParallelAction(
                driveAction, armAction, bot.getPIDAction()
                )
        );
    }
}



