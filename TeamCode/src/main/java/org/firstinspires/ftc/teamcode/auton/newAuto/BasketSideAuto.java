package org.firstinspires.ftc.teamcode.auton.newAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@Autonomous(name = "BasketSideAuto", group = "Autonomous", preselectTeleOp = "TeleopV2")
public class BasketSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Starting position of the robot (x = -11.8, y = -61.7, heading = -90 degrees)
        Pose2d initialPose = new Pose2d(-15, -62, Math.toRadians(270)); // 90
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot bot = new Robot(hardwareMap);

        // Define trajectory using Pose2d for simultaneous right and forward movement
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-8,-39)) // -8, -45
                .waitSeconds(1.4)
                .strafeTo(new Vector2d(-8, -49))
                //Arm to high speci and back down
                .strafeToLinearHeading(new Vector2d(-49,-48), Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-48.25,-36), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-51,-52), Math.toRadians(45))
                .waitSeconds(7.4)
                //intake
                .strafeToLinearHeading(new Vector2d(-58,-45), Math.toRadians(90))
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-19,-9.5, Math.toRadians(180)), Math.toRadians(0));


        // Final action to close out the trajectory
        Action trajectoryActionCloseOut = tab1.fresh().build();

        Action waitAndArm = drive.actionBuilder(initialPose)
                .afterTime(0.01, bot.setPidVals(2200,952)) // 1050, 3800 arm out
//                .afterTime(0.05, bot.intake(-0.5))

                .afterTime(1, telemetryPacket -> {//score specimen/outake
                    bot.grippy.setPosition(0.5);
                    return false;
                })
                .afterTime(0.7, telemetryPacket -> {
                    bot.flippy.setPosition(0.55);
                    return false;
                })
                .afterTime(2, bot.setPidVals(2200,600))
                .afterTime(3.2, telemetryPacket -> {
                    bot.flippy.setPosition(0.55);
                    return false;
                })
                .afterTime(3.8, telemetryPacket -> {
                    bot.grippy.setPosition(0.5);
                    return false;
                })
                .afterTime(4.8, bot.setPidVals(700,0))
                .afterTime(5.8, telemetryPacket -> {
                    bot.grippy.setPosition(0);
                    return false;
                })

                .afterTime(6.4, bot.setPidVals(0,0))
                .afterTime(6.9, telemetryPacket -> {
                    bot.grippy.setPosition(0);
                    return false;
                })
                .afterTime(6, telemetryPacket -> {
                    bot.flippy.setPosition(1);
                    return false;
                })
                .afterTime(8.2, telemetryPacket -> {
                    bot.flippy.setPosition(0.3);
                    return false;
                })
                .afterTime(8.3, telemetryPacket -> {
                    bot.flippy.setPosition(0.55);
                    return false;
                })
                .afterTime(9.7,bot.setPidVals(2000,500))
                .afterTime(12.1, bot.setPidVals(2000,6500))
                .afterTime(14.2, telemetryPacket -> {
                    bot.grippy.setPosition(0.25);
                    return false;
                })
                .afterTime(14.4, telemetryPacket -> {
                    bot.flippy.setPosition(0);
                    return false;
                })
                .afterTime(14.41, bot.setPidVals(2000,0))
                .afterTime(14.8, bot.setPidVals(0,0))

                .afterTime(27.2, telemetryPacket -> {
                    bot.flippy.setPosition(1);
                    bot.rightHang.setPosition(0.4);
                    bot.leftHang.setPosition(0.4);

                    return false;
                })
                .build();

        bot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the start of the op mode

        bot.flippy.setPosition(1);

        bot.rightHang.setPosition(0.9);
        bot.leftHang.setPosition(0.9);
        Actions.runBlocking(bot.setPidVals(1000, 0));
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