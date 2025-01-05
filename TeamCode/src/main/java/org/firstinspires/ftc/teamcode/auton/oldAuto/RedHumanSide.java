package org.firstinspires.ftc.teamcode.auton.oldAuto;

import android.util.Log;

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
@Autonomous(name = "RedHumanSide", group = "Autonomous", preselectTeleOp = "TeleopV1")
public class RedHumanSide extends LinearOpMode {
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
                .strafeToLinearHeading(new Vector2d(45, -51.5), Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(45.01, -53.01), Math.toRadians(270))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(45, -58))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(4,-38))
                .waitSeconds(3)
                .strafeTo(new Vector2d(4, -49))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(60,-57.5));
                //Arm to high speci and back down
//                .strafeToLinearHeading(new Vector2d(30,-48), Math.toRadians(75))
//                .strafeToLinearHeading(new Vector2d(38,-14), Math.toRadians(80))
//                .strafeToLinearHeading(new Vector2d(45,-12), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(45,-50), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(45,-13), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(56,-13), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(56,-50), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(56,-13), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(61,-13), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(61,-50), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(48,-54), Math.toRadians(90))
//                .waitSeconds(1.5)
//                .strafeToLinearHeading(new Vector2d(4,-45), Math.toRadians(90))
//                .waitSeconds(1.9)
//                .strafeTo(new Vector2d(48,-54))
//                .waitSeconds(1.5)
//                .strafeTo(new Vector2d(4,-45))
//                .waitSeconds(1.9)
//                .strafeToLinearHeading(new Vector2d(42,-50), Math.toRadians(90));

        // Final action to close out the trajectory
        Action trajectoryActionCloseOut = tab1.fresh().build();

        
        Action waitAndArm = drive.actionBuilder(initialPose)
                .afterTime(0.01, bot.setPidVals(2200,952)) // 1050, 3800 arm out
//                .afterTime(0.05, bot.intake(-0.5))
                .afterTime(1, telemetryPacket -> {//score specimen/outake
                    bot.intakeLeft.setPower(0.5);
                    bot.intakeRight.setPower(-0.5);
                    return false;
                })
                .afterTime(0.7, telemetryPacket -> {
                    bot.wrist.setPosition(0.55);
                    return false;
                })
                .afterTime(2, bot.setPidVals(2200,600))
                .afterTime(3.2, telemetryPacket -> {
                    bot.wrist.setPosition(0.55);
                    return false;
                })
                .afterTime(3.8, telemetryPacket -> {
                    bot.intakeLeft.setPower(-0.5);
                    bot.intakeRight.setPower(0.5);
                    return false;
                })
                .afterTime(4.8, bot.setPidVals(700,0))
                .afterTime(5.8, telemetryPacket -> {
                    bot.intakeLeft.setPower(0);
                    bot.intakeRight.setPower(0);
                    return false;
                })

                .afterTime(6.4, bot.setPidVals(0,0))
                .afterTime(6.9, telemetryPacket -> {
                    bot.wrist.setPosition(1);
                    return false;
                })
                .afterTime(13, bot.setPidVals(765,0))
                .afterTime(11.1, telemetryPacket -> {
                    bot.wrist.setPosition(0);
                    return false;
                })
                .afterTime(13.2, telemetryPacket -> {
                    bot.intakeLeft.setPower(1);
                    bot.intakeRight.setPower(-1);
                    return false;
                })
                .afterTime(22, bot.setPidVals(2200,952)) // 1050, 3800 arm out
//                .afterTime(0.05, bot.intake(-0.5))
                .afterTime(22.02, telemetryPacket -> {
                    bot.wrist.setPosition(0.55);
                    return false;
                })
                .afterTime(23, telemetryPacket -> {//score specimen/outake
                    bot.intakeLeft.setPower(0.5);
                    bot.intakeRight.setPower(-0.5);
                    return false;
                })
                .afterTime(24.3, telemetryPacket -> {
                    bot.wrist.setPosition(0.55);
                    return false;
                })
                .afterTime(24, bot.setPidVals(2180,600))
                .afterTime(25.2, telemetryPacket -> {
                    bot.wrist.setPosition(0.55);
                    return false;
                })
                .afterTime(25.8, telemetryPacket -> {
                    bot.intakeLeft.setPower(-0.5);
                    bot.intakeRight.setPower(0.5);
                    return false;
                })
                .afterTime(26.8, bot.setPidVals(700,0))
                .afterTime(27.8, telemetryPacket -> {
                    bot.intakeLeft.setPower(0);
                    bot.intakeRight.setPower(0);
                    bot.wrist.setPosition(1);
                    return false;
                })

                .afterTime(28, bot.setPidVals(0,0))
                .afterTime(28.5, telemetryPacket -> {
                    bot.wrist.setPosition(1);
                    return false;
                })
                .build();

        bot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bot.wrist.setPosition(1);
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
