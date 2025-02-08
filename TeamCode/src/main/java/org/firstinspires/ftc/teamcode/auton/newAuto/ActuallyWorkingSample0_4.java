package org.firstinspires.ftc.teamcode.auton.newAuto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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

        //TODO: spline(53.2, 54.16, -134.88) 45,  also slides fully, flip wrist later
        //TODO: aftertime 0 wrist down open grip
        //TODO: after open flip wrist down
        //TODO: spline(48.97, 38.35, -90) -90
        //TODO: when reach pos, flip down delay grip close
        //TODO: back to spline(53.2, 54.16, -134.88) 45 same arm as soon as grab
        //TODO: same deposit macro
        //TODO: line(59.36, 37.96, -89.75)
        //TODO: same scoring
        //TODO: spline(61.4, 36.2, -48.4) tangent -45


        Action driveAction = drive.actionBuilder(beginPose)
                .afterTime(0, telemetryPacket -> {
                    bot.setPidValues(1875, 0);
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.setPidValues(1875, 4700);
                    return false;
                })
                //TODO: pick up first sample
                .strafeToSplineHeading(new Vector2d(53.2, 54.16), Math.toRadians(-134.88))
                .afterTime(0.4, telemetryPacket -> {
                    bot.flippy.setPosition(0.828);
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(0.7, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(0.9, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })
                .waitSeconds(1)
                //TODO: pickup second sample
                .strafeToSplineHeading(new Vector2d(48.97, 38.35), Math.toRadians(-90))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .waitSeconds(1)
                .afterTime(0, telemetryPacket -> {
                    bot.sampleDeposit();
                    return false;
                })
                //TODO: score second sample
                .strafeToSplineHeading(new Vector2d(53.2, 54.16), Math.toRadians(-134.88))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.828);
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(0.15, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })
                .waitSeconds(1)
                //TODO: pick up third sample
                .strafeToSplineHeading(new Vector2d(59.36, 37.96), Math.toRadians(-89.75))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .waitSeconds(1)
                .afterTime(0, telemetryPacket -> {
                    bot.sampleDeposit();
                    return false;
                })
                //TODO: score third sample
                .strafeToSplineHeading(new Vector2d(53.2, 54.16), Math.toRadians(-134.88))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.828);
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(0.15, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })
                .waitSeconds(1)

                //TODO: pickup fourth sample
                .afterTime(0, telemetryPacket -> {
                    bot.setPidValues(0, 400);
                    return false;
                })
                .strafeToSplineHeading(new Vector2d(61.4, 36.2), Math.toRadians(-48.4))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.4);
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.grippyClose();
                    return false;
                })
                .waitSeconds(1)
                .afterTime(0, telemetryPacket -> {
                    bot.sampleDeposit();
                    return false;
                })

                //TODO: score fourth sample
                .strafeToSplineHeading(new Vector2d(53.2, 54.16), Math.toRadians(-134.88))
                .afterTime(0, telemetryPacket -> {
                    bot.flippy.setPosition(0.828);
                    bot.grippyOpen();
                    return false;
                })
                .afterTime(0.15, telemetryPacket -> {
                    bot.flippy.setPosition(0.7);
                    return false;
                })
                .afterTime(0.5, telemetryPacket -> {
                    bot.fullRetract();
                    return false;
                })



                .build();

        Actions.runBlocking(
                new ParallelAction(
                        driveAction, bot.getPIDAction()
                )
        );
    }
}
