package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Disabled
@Autonomous
@Config
public class NoRRvsRR extends LinearOpMode {
    public static boolean RR = false;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        waitForStart();

        if (RR) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
            Action moveForward = drive.actionBuilder(new Pose2d(0,0,0))
                    .lineToX(96)
                    .build();
            Actions.runBlocking(moveForward);
        }
        else {
            timer.reset();
            while (timer.seconds() < 4) {
                bot.leftFront.setPower(0.6);
                bot.leftBack.setPower(0.6);
                bot.rightBack.setPower(0.6);
                bot.rightFront.setPower(0.6);
            }
            bot.leftFront.setPower(0);
            bot.leftBack.setPower(0);
            bot.rightBack.setPower(0);
            bot.rightFront.setPower(0);
        }

    }
}
