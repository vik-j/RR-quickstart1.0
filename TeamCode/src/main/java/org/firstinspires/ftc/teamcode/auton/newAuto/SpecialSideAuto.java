package org.firstinspires.ftc.teamcode.auton.newAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
        Pose2d beginPose = new Pose2d(-15, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Robot bot = new Robot(hardwareMap);

        drive.noCorrection();

        bot.grippyClose();
        bot.flippy.setPosition(1);
        bot.twisty.setPosition(0);
        bot.resetEncoders();

        waitForStart();

        Robot.stopPid = false;

        Action firstSpeci = bot.autoAction(drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(-8.5, 36.5))
                .waitSeconds(0.5)
                .build(), bot.speciDeposit());

        Action firstSpeciHook = bot.pid(bot.speciDeposit2());

        Action retract = bot.pid(bot.retract());

        //TODO: fix actionBuilder poses not updating

        Action pushBlocks = drive.actionBuilder(MecanumDrive.lastPose)
                .splineToSplineHeading(new Pose2d(-32.5515,44.2, Math.toRadians(270)), Math.toRadians(180))
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(-46, 13), Math.toRadians(180))
                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(-46, 49.5), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .waitSeconds(0.001)
                .splineToConstantHeading(new Vector2d(-56, 13), Math.toRadians(180))
                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(-56, 45), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .waitSeconds(0.001)
                .splineToConstantHeading(new Vector2d(-63, 13), Math.toRadians(180))
                .waitSeconds(0.001)
                .stopAndAdd(new SequentialAction(new InstantAction(drive::enableHeadingCorrection)
                        ,new InstantAction(() -> drive.enableTranslationalCorrection(2.0))
                ,bot.setPidVals(2300, 0), new InstantAction(() -> bot.flippy.setPosition(0.97))))
                .strafeToConstantHeading(new Vector2d(-63, 53.25), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
                .waitSeconds(0.3)
                .build();

        Action grabSecondSpeci = new SequentialAction(new InstantAction(bot::badClose), new SleepAction(0.4));

        Action scoreSecondSpeci = drive.actionBuilder(MecanumDrive.lastPose)
                .stopAndAdd(new InstantAction(() -> bot.setPidValues(2300, 500)))
                .strafeTo(new Vector2d(0, 49))
                .stopAndAdd(new InstantAction(bot::newSpeciPivot))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(0, 36.5), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
                .waitSeconds(1.5)
                .build();


        Action armAction = drive.actionBuilder(beginPose)
                .afterTime(0.01, telemetryPacket -> {
                    bot.newSpeci();
                    return false;
                })
                .afterTime(1.4, telemetryPacket -> {
                    bot.newSpeci2();
                    return false;
                })
                .afterTime(1.8, telemetryPacket -> {
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
                    bot.specPickup();
                    return false;
                })
                .afterTime(12, telemetryPacket -> {
                    bot.badClose();
                    return false;
                })
                .afterTime(12.4, telemetryPacket -> {
                    bot.setPidValues(2300,500);
                    return false;
                })
                .afterTime(12.9, telemetryPacket -> {
                    bot.newSpeciPivot();
                    return false;
                })
                .afterTime(15.2, telemetryPacket -> {
                    bot.newSpeciSlides();
                    return false;
                })
                .afterTime(16.45, telemetryPacket -> {
                    bot.newSpeci2();
                    return false;
                })
                .afterTime(16.95, telemetryPacket -> {
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(17, telemetryPacket -> {
                    bot.specPickup();
                    return false;
                })
                .afterTime(19.45, telemetryPacket -> {
                    bot.badClose();
                    return false;
                })
                .afterTime(20.05, telemetryPacket -> {
                    bot.newSpeci();
                    return false;
                })
                .afterTime(21.8, telemetryPacket -> {
                    bot.newSpeci2();
                    return false;
                })
//                .afterTime(14.5, telemetryPacket -> {
//                    bot.speciMacro();
//                    return false;
//                })

                .build();

        //TODO: 13.38 V

        Action driveAction = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(-8.5, 36.5))
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-32.5515,44.2, Math.toRadians(270)), Math.toRadians(180))
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(-46, 13), Math.toRadians(180))
                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(-46, 49.5), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .waitSeconds(0.001)
                .splineToConstantHeading(new Vector2d(-56, 13), Math.toRadians(180))
                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(-56, 45), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .waitSeconds(0.001)
                .splineToConstantHeading(new Vector2d(-63, 13), Math.toRadians(180))
                .waitSeconds(0.001)
                .stopAndAdd(new SequentialAction(new InstantAction(drive::enableHeadingCorrection)
                        ,new InstantAction(() -> drive.enableTranslationalCorrection(2.0))))
                .strafeToConstantHeading(new Vector2d(-63, 53.25), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(0, 49))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(0, 36.5), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(-36, 49), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(0, 36), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80,80))

//                .waitSeconds(0.5)
//                .strafeToConstantHeading(new Vector2d(-36, 54))
//                .waitSeconds(0.5)
//                .strafeToConstantHeading(new Vector2d(-2, 36))
//                .waitSeconds(0.5)
//                .strafeToConstantHeading(new Vector2d(-36, 54))
//                .waitSeconds(0.5)
//                .strafeToConstantHeading(new Vector2d(3, 36))
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



