package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@TeleOp
public class PidfTuner extends OpMode {
    private PIDController armController, slideController;

    public static double fP = 0.008, fI = 0, fD = 0;  //fD = 0.00001, fP = 0.002
    public static double fF = 0.01; //fF = 0.0022
    public static double sP = 0.005, sI, sD;
    public static double sF;

    public static boolean PIDon = false;

    public static double grippyPos = 0;
    public static double twistyPos = 0;
    public static double flippyPos = 0;

    public static int armTarget = 0;
    public static int slideTarget = 0;
    public static double servoTarget = 0.5;
    
    public static double multiplier = 0.01;

    private final double ticks_in_degree = 1850 / 90.0;

    private DcMotorEx flip, slide;
//    private Servo wrist;
    MecanumDrive drive;
    Robot bot;
    Pose2d beginPose = new Pose2d(15, -62, Math.toRadians(270));

    @Override
    public void init() {
        bot = new Robot(hardwareMap);

        armController = new PIDController(fP, fI, fD);
        slideController = new PIDController(sP,sI,sD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, beginPose);

        flip = (DcMotorEx) bot.flip;
        slide = (DcMotorEx) bot.slide;
//        wrist = bot.wrist;

        flip.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        flip.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        int armPos, slidePos;

        armPos = flip.getCurrentPosition();

        slidePos = slide.getCurrentPosition();

        if (PIDon) {

            armController.setPID(fP, fI, fD);

            double pid = armController.calculate(armPos, armTarget);
            double ff = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * fF;
    //        if (armPos > 0) pid *= Math.cos(Math.toRadians(armPos/ticks_in_degree));

            double power = pid + ff;

            flip.setPower(power);

            slideController.setPID(sP,sI,sD);
            slidePos = slide.getCurrentPosition();
            double pid2 = slideController.calculate(slidePos, slideTarget);

            slide.setPower(pid2);

            telemetry.addData("flipPower", power);

            bot.flippy.setPosition(bot.scaleFlippy(flippyPos));
            bot.twisty.setPosition(twistyPos);
            bot.grippy.setPosition(grippyPos);

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

//        wrist.setPosition(servoTarget);

        telemetry.addData("armPos", armPos);
        telemetry.addData("slidePos", slidePos);
        telemetry.addData("slidePower", slide.getPower());
        telemetry.update();
    }

}

