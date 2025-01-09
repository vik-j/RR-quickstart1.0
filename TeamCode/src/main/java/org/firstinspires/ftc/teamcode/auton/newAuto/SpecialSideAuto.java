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
        Pose2d beginPose = new Pose2d(15, -62, Math.toRadians(90));
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
                .afterTime(1.8, telemetryPacket -> {
                    bot.grippyOpen();
                    bot.reset();
                    return false;
                })
                .build();

        Actions.runBlocking(
                new ParallelAction(
                drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(7, -26), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(36,-36, Math.toRadians(180)), Math.toRadians(0))
                        .build(), armAction, bot.getPIDAction()
                )
        );
    }
}



