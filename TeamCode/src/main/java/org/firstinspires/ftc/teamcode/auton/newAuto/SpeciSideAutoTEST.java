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
@Autonomous(name = "SpeciSideAutoTEST", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class SpeciSideAutoTEST extends LinearOpMode {
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



        //TODO: 13.38 V

        Action driveAction = drive.actionBuilder(beginPose)
                .afterTime(0.01, telemetryPacket -> {
                    bot.newSpeci();
                    return false;
                })
                .strafeToConstantHeading(new Vector2d(-8.5, 36.5))
                .afterTime(0, telemetryPacket -> {
                    bot.newSpeci2();
                    return false;
                })
                .waitSeconds(0.5)
                .afterTime(0, telemetryPacket -> {
                    bot.reset();
                    return false;
                })
                .splineToSplineHeading(new Pose2d(-32.5515,44.2, Math.toRadians(270)), Math.toRadians(180))
//                .waitSeconds(0.2)
//                .splineToConstantHeading(new Vector2d(-46, 13), Math.toRadians(180))
//                .waitSeconds(0.001)
//                .strafeToConstantHeading(new Vector2d(-46, 49.5), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
//                .waitSeconds(0.001)
//                .splineToConstantHeading(new Vector2d(-56, 13), Math.toRadians(180))
//                .waitSeconds(0.001)
//                .strafeToConstantHeading(new Vector2d(-56, 45), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
//                .waitSeconds(0.001)
//                .splineToConstantHeading(new Vector2d(-63, 13), Math.toRadians(180))
//                .waitSeconds(0.001)
//                .stopAndAdd(new SequentialAction(new InstantAction(drive::enableHeadingCorrection)
//                        ,new InstantAction(() -> drive.enableTranslationalCorrection(2.0))))
//                .strafeToConstantHeading(new Vector2d(-63, 53.25), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
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
                        driveAction, bot.getPIDAction()
                )
        );
    }
}



