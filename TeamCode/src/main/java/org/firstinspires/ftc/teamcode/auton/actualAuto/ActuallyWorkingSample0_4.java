package org.firstinspires.ftc.teamcode.auton.actualAuto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BasketSideMecanumDrive;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Autonomous(name = "🟨🐖", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class ActuallyWorkingSample0_4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(39, 62.9375, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Robot bot = new Robot(hardwareMap);

        bot.resetEncoders();

        drive.enableHeadingCorrection();
        drive.enableTranslationalCorrection(1.0);
        drive.setCorrectionTimeout(0.75);

        bot.grippyClose();
        bot.flippy.setPosition(1);
        bot.twisty.setPosition(0);
        bot.resetEncoders();

        waitForStart();


        Action driveAction = drive.actionBuilder(beginPose)
                .afterTime(0, telemetryPacket -> {
                    bot.setPidValues(1875, 0);
                    bot.twisty.setPosition(1);
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.setPidValues(1875, 4700);
                    return false;
                })
                //TODO: score first sample
                .strafeToSplineHeading(new Vector2d(51.2, 52.16), Math.toRadians(-134.88))
                .afterTime(1.3, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(1.45, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(1.55, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(1.6, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })
                .waitSeconds(0.6)
                .strafeToSplineHeading(new Vector2d(53.6, 54.56), Math.toRadians(-134.88))
                .waitSeconds(0.8)
                //TODO: pickup second sample
                .afterTime(0, telemetryPacket -> {
                    bot.setPidValues(0, 600);
                    return false;
                })
                .waitSeconds(0.4)
                .strafeToSplineHeading(new Vector2d(48.9, 41.75), Math.toRadians(-90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-30, 30))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.3, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .waitSeconds(0.8)
                .afterTime(0, telemetryPacket -> {
                    bot.samplePivot();
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.sampleSlides();
                    return false;
                })
                //TODO: score second sample
                .strafeToSplineHeading(new Vector2d(50.7, 51.86), Math.toRadians(-134.88))
                .afterTime(1.1, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    bot.sweepyUp();
                    return false;
                })
                .afterTime(1.25, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(1.75, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(1.9, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })
                .waitSeconds(0.75)
                .strafeToSplineHeading(new Vector2d(54.1, 55.06), Math.toRadians(-134.88))
                .waitSeconds(0.8)
                //TODO: pick up third sample
                .afterTime(0, telemetryPacket -> {
                    bot.setPidValues(0, 200);
                    return false;
                })
                .strafeToSplineHeading(new Vector2d(58.91, 40.5), Math.toRadians(-89.75), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.3, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .waitSeconds(1)
                .afterTime(0, telemetryPacket -> {
                    bot.samplePivot();
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.sampleSlides();
                    return false;
                })
                //TODO: score third sample
                .strafeToSplineHeading(new Vector2d(51.7, 52.66), Math.toRadians(-134.88))
                .afterTime(0.9, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(1.75, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(1.9, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(2.05, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })
                .waitSeconds(0.75)
                .strafeToSplineHeading(new Vector2d(54.1, 55.06), Math.toRadians(-134.88))
                .waitSeconds(0.8)
                //TODO: pickup fourth sample
                .afterTime(0.5, telemetryPacket -> {
                    bot.setPidValues(0, 600);
                    bot.twisty.setPosition(bot.scaleTwisty(0.74));
                    return false;
                })
                .strafeToSplineHeading(new Vector2d(61.2, 40.48), Math.toRadians(-52.5), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.3, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .waitSeconds(0.8)
                .afterTime(0, telemetryPacket -> {
                    bot.samplePivot();
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.sampleSlides();
                    return false;
                })

                //TODO: score fourth sample
                .strafeToSplineHeading(new Vector2d(50.2, 51.16), Math.toRadians(-134.88))
                .afterTime(1.1, telemetryPacket -> {
                    bot.flippy.setPosition(0.9);
                    return false;
                })
                .afterTime(1.25, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(1.65, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(1.75, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })
                .waitSeconds(0.6)
                .strafeToSplineHeading(new Vector2d(54.1, 55.06), Math.toRadians(-134.88))
                .waitSeconds(0.8)



                .build();

        Actions.runBlocking(
                new ParallelAction(
                        driveAction, bot.getPIDAction()
                )
        );
    }
}
