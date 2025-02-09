package org.firstinspires.ftc.teamcode.auton.newAuto;

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

@Autonomous(name = "💉🐖", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class ActuallyWorkingSample0_4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(40, 64, Math.toRadians(180));
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
                //TODO: score up first sample
                .strafeToSplineHeading(new Vector2d(51.2, 52.16), Math.toRadians(-134.88))
                .afterTime(1.1, telemetryPacket -> {
                    bot.flippy.setPosition(0.828);
                    return false;
                })
                .afterTime(1.25, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(1.45, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(1.5, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(53.6, 54.56), Math.toRadians(-134.88))
                .waitSeconds(0.4)
                //TODO: pickup second sample
                .afterTime(0, telemetryPacket -> {
                    bot.setPidValues(0, 400);
                    return false;
                })
                .strafeToSplineHeading(new Vector2d(48.97, 39.85), Math.toRadians(-90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-30, 30))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.3, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .waitSeconds(0.6)
                .afterTime(0, telemetryPacket -> {
                    bot.samplePivot();
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.sampleSlides();
                    return false;
                })
                //TODO: score second sample
                .strafeToSplineHeading(new Vector2d(50.2, 51.16), Math.toRadians(-134.88))
                .afterTime(1.1, telemetryPacket -> {
                    bot.flippy.setPosition(0.828);
                    return false;
                })
                .afterTime(1.25, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(1.45, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(1.5, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(53.6, 54.56), Math.toRadians(-134.88))
                .waitSeconds(0.4)
                //TODO: pick up third sample
                .afterTime(0, telemetryPacket -> {
                    bot.setPidValues(0, 400);
                    return false;
                })
                .strafeToSplineHeading(new Vector2d(59.36, 39.56), Math.toRadians(-89.75), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-30, 30))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.3, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .waitSeconds(0.6)
                .afterTime(0, telemetryPacket -> {
                    bot.samplePivot();
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.sampleSlides();
                    return false;
                })
                //TODO: score third sample
                .strafeToSplineHeading(new Vector2d(51.2, 52.16), Math.toRadians(-134.88))
                .afterTime(1.1, telemetryPacket -> {
                    bot.flippy.setPosition(0.828);
                    return false;
                })
                .afterTime(1.25, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(1.45, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(1.5, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(53.6, 54.56), Math.toRadians(-134.88))
                .waitSeconds(0.4)
                //TODO: pickup fourth sample
                .afterTime(0, telemetryPacket -> {
                    bot.setPidValues(0, 400);
                    bot.twisty.setPosition(0.76);
                    return false;
                })
                .strafeToSplineHeading(new Vector2d(61.4, 37.2), Math.toRadians(-48.4), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-30, 30))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.3, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .waitSeconds(0.6)
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
                    bot.flippy.setPosition(0.828);
                    return false;
                })
                .afterTime(1.25, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(1.45, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(1.5, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(53.6, 54.56), Math.toRadians(-134.88))
                .waitSeconds(0.4)



                .build();

        Actions.runBlocking(
                new ParallelAction(
                        driveAction, bot.getPIDAction()
                )
        );
    }
}
