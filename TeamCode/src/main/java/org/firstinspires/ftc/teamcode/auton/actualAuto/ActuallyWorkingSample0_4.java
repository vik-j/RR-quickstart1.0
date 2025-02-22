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
import org.firstinspires.ftc.teamcode.auton.qolActions.qol;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Autonomous(name = "🟨🐖", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class ActuallyWorkingSample0_4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(39, 62.9375, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Robot bot = new Robot(hardwareMap);

        bot.resetEncoders();
        qol q = new qol(bot);

        drive.enableHeadingCorrection();
        drive.enableTranslationalCorrection(1.0);
        drive.setCorrectionTimeout(0.75);

        bot.grippyClose();
        bot.flippy.setPosition(bot.scaleFlippy(1));
        bot.twisty.setPosition(0);
        bot.resetEncoders();

        waitForStart();


        Action driveAction = drive.actionBuilder(beginPose)
                .afterTime(0, q.combine(q.arm(1875, 0), q.twisty(0), q.flippy(0.7)))
                .afterTime(0.5, q.arm(1900, 4600))
                //TODO: score first sample
                .strafeToSplineHeading(new Vector2d(51.2, 52.16), Math.toRadians(-134.88))
                .afterTime(1.3, q.flippy(0.9))
                .afterTime(1.45, q.grippyOpen())
                .afterTime(1.95, q.flippy(0.7))
                .afterTime(2.05, q.arm(1875, 0))
                .afterTime(2.3, q.arm(0,0))
                .waitSeconds(0.4)
                .strafeToSplineHeading(new Vector2d(56.6, 57.56), Math.toRadians(-134.88))
                .waitSeconds(0.8)
                //TODO: pickup second sample
                .afterTime(0, q.arm(0, 600))
                .waitSeconds(0.4)
                .strafeToSplineHeading(new Vector2d(48.9, 41.75), Math.toRadians(-90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-30, 30))
                .afterTime(0, q.flippy(0.4))
                .afterTime(0.13, q.grippyClose())
                .waitSeconds(0.3)
                .afterTime(0, q.samplePivot())
                .afterTime(0.5, q.sampleSlides())
                //TODO: score second sample
                .strafeToSplineHeading(new Vector2d(51.7, 52.86), Math.toRadians(-134.88))
                .afterTime(1.6, q.combine(q.flippy(0.9), q.sweepyUp()))
                .afterTime(1.75, q.grippyOpen())
                .afterTime(2.25, q.flippy(0.7))
                .afterTime(2.6, q.arm(1875, 0))
                .afterTime(3, q.arm(0,0))
                .waitSeconds(0.9)
                .strafeToSplineHeading(new Vector2d(57.1, 58.06), Math.toRadians(-134.88))
                .waitSeconds(1.3)
                //TODO: pick up third sample
                .afterTime(0, q.arm(0, 475))
                .strafeToSplineHeading(new Vector2d(58.91, 40.25), Math.toRadians(-89.75), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .afterTime(0, q.flippy(0.4))
                .afterTime(0.3, q.grippyClose())
                .waitSeconds(1)
                .afterTime(0, q.samplePivot())
                .afterTime(0.5, q.sampleSlides())
                //TODO: score third sample
                .strafeToSplineHeading(new Vector2d(52.7, 53.66), Math.toRadians(-134.88))
                .afterTime(1.8, q.flippy(0.9))
                .afterTime(1.95, q.grippyOpen())
                .afterTime(2.2, q.flippy(0.7))
                .afterTime(2.75, q.arm(1875, 0))
                .afterTime(2.95, q.arm(0,0))
                .waitSeconds(0.8)
                .strafeToSplineHeading(new Vector2d(56.1, 57.06), Math.toRadians(-134.88))
                .waitSeconds(1.4)
                //TODO: pickup fourth sample
                .afterTime(0.5, q.combine(q.arm(0, 1200), q.twisty(0.28)))
                .strafeToSplineHeading(new Vector2d(59.8, 40.78), Math.toRadians(-52.5), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .afterTime(0, q.flippy(0.4))
                .afterTime(0.3, q.grippyClose())
                .waitSeconds(0.8)
                .afterTime(0, q.samplePivot())
                .afterTime(0.5, q.sampleSlides())

                //TODO: score fourth sample
                .strafeToSplineHeading(new Vector2d(51.2, 52.16), Math.toRadians(-134.88))
                .afterTime(1.3, q.flippy(0.9))
                .afterTime(1.55, q.grippyOpen())
                .afterTime(2.05, q.flippy(0.7))
                .afterTime(2.45, q.arm(1875, 0))
                .afterTime(2.65, q.arm(0,0))
                .waitSeconds(0.1)
                .strafeToSplineHeading(new Vector2d(57.1, 58.06), Math.toRadians(-134.88))
                .afterTime(0, q.hangUp())
                .afterTime(0.2, q.hangDown())
                .afterTime(0.4, q.hangUp())
                .afterTime(0.6, q.hangDown())
                .afterTime(0.8, q.hangUp())
                .afterTime(1, q.hangDown())
                .afterTime(1.2, q.hangUp())
                .waitSeconds(1.4)
                .afterTime(0, q.combine(q.flippy(1), q.arm(1100, 2000)))
                .splineToLinearHeading(new Pose2d(36, 12.5, Math.toRadians(180)), Math.toRadians((180)))
                .splineToLinearHeading(new Pose2d(15, 12.5, Math.toRadians(180)), Math.toRadians((180)))

                .build();

        Actions.runBlocking(
                new ParallelAction(
                        driveAction, bot.getPIDAction()
                )
        );
    }
}
