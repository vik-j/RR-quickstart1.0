package org.firstinspires.ftc.teamcode.auton.compAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.auton.qolActions.qol;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@Autonomous(name = "Gold2Blue5_0", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class Gold2Blue5_0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-15, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Robot bot = new Robot(hardwareMap);
        qol q = new qol(bot);

        drive.enableHeadingCorrection();
        drive.enableTranslationalCorrection(2.0);
        drive.setCorrectionTimeout(0.5);

        //TODO: AUTO TUNED TO OLD BLUE SIDE AT HOME FIELD
        //TODO: Positive offset is amount pushed away from sub

        double wallOffset = Robot.CompFieldOffsets.Gold2Blue - 0.5;

        //TODO: IDEAL DISTANCE TO BAR FROM FRONT OF ROBOT: 32.56 inches
        //TODO: Diag to wall 49 inch. bar at sub bottom 2 inch tall



        bot.grippyClose();
        bot.flippy.setPosition(1);
        bot.twisty.setPosition(0);
        bot.resetEncoders();
        bot.sweepyUp();


        waitForStart();

        Robot.stopPid = false;



        //TODO: 13.38 V

        Action driveAction = drive.actionBuilder(beginPose)
                //TODO: 1st speci

                .afterTime(0.01, q.firstSpeci())
                .strafeToConstantHeading(new Vector2d(-8.5, 34.4 + wallOffset))
                .afterTime(0, q.firstSpeci2())
                .waitSeconds(0.2)
                .afterTime(0, q.grippyOpen())
                .afterTime(0.1, q.combine(q.reset(), q.flippy(1)))
                .afterTime(1.25, q.autoSamplePickup())
                .splineToSplineHeading(new Pose2d(-27.8,39.92, Math.toRadians(-145.5)), Math.toRadians(180))

                //TODO: pickup 1st sample
                // .splineToSplineHeading(new Pose2d(-28.07,39.92, Math.toRadians(-145.5)), Math.toRadians(180))
                .afterTime(0.2, q.flippy(0.4))
                .afterTime(0.42, q.grippyClose())
                .afterTime(0.9, q.flippy(0.6))
                .waitSeconds(0.3)

                //TODO: drop off first sample
                .turnTo(Math.toRadians(123.5), new TurnConstraints(20, -20, 20))
                .afterTime(0, q.grippyOpen())
                .splineToSplineHeading(new Pose2d(-38.25, 40.48, Math.toRadians(-146.44)), Math.toRadians(270))

                //TODO: pick up 2nd sample
                //.splineToSplineHeading(new Pose2d(-39.15, 40.48, Math.toRadians(-146.44)), Math.toRadians(270))
                .afterTime(0.25, q.flippy(0.4))
                .afterTime(0.47, q.grippyClose())
                .afterTime(0.9, q.flippy(0.6))

                //TODO: drop off 2nd sample
                .waitSeconds(0.35)
                .turnTo(Math.toRadians(121.95), new TurnConstraints(20, -20, 20))
                .afterTime(0, q.combine(q.grippyOpen(), new InstantAction(() -> drive.setCorrectionTimeout(1.25))))
                .splineToSplineHeading(new Pose2d(-46.85, 39.5, Math.toRadians(-151.7)), Math.toRadians(270))

                //TODO: pick up 3rd sample
                // .splineToSplineHeading(new Pose2d(-45.97, 39.097, Math.toRadians(-151.7)), Math.toRadians(270))
                .afterTime(0.3, q.flippy(0.4))
                .afterTime(0.52, q.combine(q.grippyClose(), new InstantAction(() -> drive.setCorrectionTimeout(1))))
                .afterTime(1.05, q.combine(q.twisty(0.75), q.flippy(0.6)))
                .afterTime(1.3, q.combine(q.reset(), q.flippy(0.6)))
                .waitSeconds(0.45)
                .afterTime(0.6, q.arm(0, 800))

                //TODO: drop off 3rd sample
                .strafeToSplineHeading(new Vector2d(-39.2, 48.193), Math.toRadians(120.3))
                .afterTime(0, q.grippyOpen())


                .afterTime(0.75, q.specimenPickup())
                .waitSeconds(0.3)
                //TODO: pickup 2nd speci
                .strafeToLinearHeading(new Vector2d(-35.52, 47), Math.toRadians(-90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-35.52, 54), Math.toRadians(-90))
                .afterTime(0.25, q.grippyClose())
                .afterTime(0.7, q.flippy(0.8))
                .waitSeconds(0.15)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.3, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.05, q.specimenDeposit())
                //TODO: score 2nd speci
                .strafeToConstantHeading(new Vector2d(-3, 34.4 + wallOffset))

                .afterTime(0.2, q.specimenDeposit2())
                .afterTime(0.5, q.grippyOpen())
                .waitSeconds(0.1)


                .afterTime(0.75, q.specimenPickup())
                //TODO: pickup 3rd speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53.5), Math.toRadians(-90))
                .afterTime(0, q.grippyClose())
                .afterTime(0.6, q.flippy(0.8))
                .waitSeconds(0)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.35, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.2, q.specimenDeposit())

                //TODO: score 3rd speci
                .strafeToConstantHeading(new Vector2d(0, 34.14 + wallOffset))

                .afterTime(0.2, q.specimenDeposit2())
                .afterTime(0.45, q.grippyOpen())
                .waitSeconds(0)


                .afterTime(0.75, q.specimenPickup())

                //TODO: pick up 4th speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53), Math.toRadians(-90))
                .afterTime(0, q.grippyClose())
                .afterTime(0.6, q.flippy(0.7))
                .waitSeconds(0)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.4, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.37, q.specimenDeposit())
                //TODO: drop off 4th speci
                .strafeToConstantHeading(new Vector2d(5, 33.9 + wallOffset))
                .afterTime(0.3, q.specimenDeposit2())
                .afterTime(0.65, q.grippyOpen())
                .waitSeconds(0.05)


                .afterTime(0.75, q.specimenPickup())


                //TODO: pick up 5th speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53.3), Math.toRadians(-90))
                .afterTime(0, q.grippyClose())
                .afterTime(0.6, q.flippy(0.7))
                .waitSeconds(0)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.4, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.32, q.specimenDeposit())
                //TODO: drop off 5th speci
                .strafeToConstantHeading(new Vector2d(7, 32.7 + wallOffset))
                .afterTime(0.15, q.specimenDeposit2())
                .afterTime(0.47 , q.grippyOpen())
                .waitSeconds(0.2)
                .afterTime(0, q.hangUp())
                .afterTime(0.2, q.hangAlmostDown())
                .afterTime(0.4, q.hangUp())

                .afterTime(2, q.reset())

                //TODO: park
                .strafeToConstantHeading(new Vector2d(-60, 60), new TranslationalVelConstraint(120), new ProfileAccelConstraint(-120, 120))

//                .waitSeconds(0.3)
//                .strafeTo(new Vector2d(0, 49))
//                .waitSeconds(0.5)
//                .strafeTo(new Vector2d(0, 36.5), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
//                .waitSeconds(1.5)
//                .strafeTo(new Vector2d(-36, 49), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
//                .waitSeconds(0.5)
//                .strafeToConstantHeading(new Vector2d(0, 36), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80,80))

                .build();

        Actions.runBlocking(
                new ParallelAction(
                        Robot.driveAction(drive, beginPose, q, wallOffset), bot.getPIDAction()
                )
        );
    }
}