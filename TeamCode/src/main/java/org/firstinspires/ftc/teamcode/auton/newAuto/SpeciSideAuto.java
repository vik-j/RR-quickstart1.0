package org.firstinspires.ftc.teamcode.auton.newAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@Autonomous(name = "SpeciSideAuto", group = "Autonomous", preselectTeleOp = "TeleopV1")
public class SpeciSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Starting position of the robot (x = -11.8, y = -61.7, heading = -90 degrees)
        Pose2d initialPose = new Pose2d(15, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot bot = new Robot(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(9.6,-39)) // -8, -45
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(9.6, -49))
                .waitSeconds(1.5)
                .splineToSplineHeading(new Pose2d(36, -40, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(39.5, -15), Math.toRadians(90))
                .strafeTo(new Vector2d(45, -15))
                .strafeToLinearHeading(new Vector2d(45, -50.5), Math.toRadians(90))
                .strafeTo(new Vector2d(45, -46))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(45.01, -53.5), Math.toRadians(270))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(45, -58), Math.toRadians(270))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(42,-56), Math.toRadians(270));

        Action trajectoryActionCloseOut = tab1.fresh().build();


        Action waitAndArm = drive.actionBuilder(initialPose)
                .afterTime(0.01, bot.setPidVals(2200,952)) // 1050, 3800 arm out
                .afterTime(2, bot.setPidVals(2200,600))
                .afterTime(4.8, bot.setPidVals(700,0))

                .afterTime(6.4, bot.setPidVals(0,0))

                .afterTime(13, bot.setPidVals(575,0))

                .afterTime(20, bot.setPidVals(900,0))
                .afterTime(22, bot.setPidVals(0,0))

                .afterTime(19, telemetryPacket -> {
                    bot.flippy.setPosition(0.35);
                    return false;
                })
                .afterTime(21.8, telemetryPacket -> {
                    bot.grippy.setPosition(0.3);
                    return false;
                })
                .afterTime(20.1, telemetryPacket -> {
                    bot.flippy.setPosition(0.35);
                    return false;
                })
                .afterTime(20.3, telemetryPacket -> {
                    bot.flippy.setPosition(0.35);
                    return false;
                })
                .afterTime(20.5, telemetryPacket -> {
                    bot.flippy.setPosition(0.35);
                    return false;
                })
                .afterTime(20.7, telemetryPacket -> {
                    bot.flippy.setPosition(0.35);
                    return false;
                })
                .afterTime(22, bot.setPidVals(650,980))
                .afterTime(25, bot.setPidVals(350, 980))
                .afterTime(22.01, telemetryPacket -> {
                    bot.flippy.setPosition(0.35);
                    return false;
                })// 1050, 3800 arm out
                .afterTime(0.05, bot.intake(-0.5))
                .afterTime(22.02, telemetryPacket -> {
                    bot.flippy.setPosition(0.35);
                    return false;
                })
                .afterTime(24.3, telemetryPacket -> {
                    bot.flippy.setPosition(0.35);
                    return false;
                })
                .afterTime(25.2, telemetryPacket -> {
                    bot.flippy.setPosition(0.35);
                    return false;
                })
                .afterTime(28.5, telemetryPacket -> {
                    bot.grippy.setPosition(0.5);
                    return false;
                })
                .afterTime(26.8, bot.setPidVals(700,0))
                .afterTime(27.8, telemetryPacket -> {
                    bot.grippy.setPosition(0);
                    bot.flippy.setPosition(1);
                    return false;
                })

                .afterTime(28, bot.setPidVals(0,0))
                .afterTime(28.5, telemetryPacket -> {
                    bot.flippy.setPosition(1);
                    return false;
                })
                .build();

        bot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bot.flippy.setPosition(1);




        // Wait for the start of the op mode
        waitForStart();

        if (isStopRequested()) return;
        Robot.stopPid = false;

        // Execute the defined trajectory
        Action trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                trajectoryActionChosen,
                                trajectoryActionCloseOut),
                        waitAndArm,
                        bot.getPIDAction()
                )
        );
        bot.stopPidAction();
    }
}

