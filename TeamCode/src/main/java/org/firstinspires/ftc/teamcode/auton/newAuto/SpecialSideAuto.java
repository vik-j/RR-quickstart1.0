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
@Autonomous(name = "SpecialSideAuto", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class SpecialSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-15, 62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Robot bot = new Robot(hardwareMap);

        bot.grippyClose();
        bot.flippy.setPosition(0);
        bot.twisty.setPosition(1);
//        Actions.runBlocking(bot.setPidVals(900, 0));
//        Actions.runBlocking(bot.pidfLoopSingular(900));

        waitForStart();

        Robot.stopPid = false;

        Action armAction = drive.actionBuilder(beginPose)
                .afterTime(0.01, telemetryPacket -> {
                    bot.speciMacro();
                    return false;
                })
                .afterTime(1.4, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
//                .afterTime(2, telemetryPacket -> {
//                    bot.samplePickup();
//                    return false;
//                })
//                .afterTime(3, bot.setPidVals(140, 1830))
//                .afterTime(3.6, telemetryPacket -> {
//                    bot.grippyClose();
//                    return false;
//                })
                .afterTime(2, telemetryPacket -> {
                    bot.reset();
                    return false;
                })
                .afterTime(10, telemetryPacket -> {
                    bot.speciPickup2();
                    return false;
                })
                .afterTime(14.5, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .afterTime(15.25, telemetryPacket -> {
                    bot.speciMacroPlus(250);
                    return false;
                })

                .build();

        Action driveAction = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(-6, 36))
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-32.5515,44.2, Math.toRadians(270)), Math.toRadians(180))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(-46, 16), Math.toRadians(180))
                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(-46, 47), new TranslationalVelConstraint(100.0), new ProfileAccelConstraint(-150, 150))
                .waitSeconds(0.001)
                .splineToConstantHeading(new Vector2d(-56, 16), Math.toRadians(180))
                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(-56, 45), new TranslationalVelConstraint(100.0), new ProfileAccelConstraint(-150, 120))
                .waitSeconds(0.001)
                .splineToConstantHeading(new Vector2d(-63.5, 16), Math.toRadians(180))
                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(-63.5, 47), new TranslationalVelConstraint(100.0), new ProfileAccelConstraint(-150, 120))
                .waitSeconds(0.1)
                .splineToLinearHeading(new Pose2d(-36, 57.5, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0.75)
                .strafeToConstantHeading(new Vector2d(-8.25, 36))
//                .waitSeconds(2)
//                .strafeToSplineHeading(new Vector2d(-34.95, 43.635), Math.toRadians(149.3))
//                .waitSeconds(2)
//                .strafeToSplineHeading(new Vector2d(-43.67, 41.8), Math.toRadians(231.777))
//                .waitSeconds(2)
//                .strafeToSplineHeading(new Vector2d(-45.02, 43.37), Math.toRadians(144.56))
//                .waitSeconds(0.5)
//                .strafeToSplineHeading(new Vector2d(52.93, -41.32), Math.toRadians(51.15))
//                .waitSeconds(0.5)
//                .strafeToSplineHeading(new Vector2d(36.481, -46.63), Math.toRadians(-90))
                .build();

        Actions.runBlocking(
                new ParallelAction(
                driveAction, armAction, bot.getPIDAction()
                )
        );
    }
}



