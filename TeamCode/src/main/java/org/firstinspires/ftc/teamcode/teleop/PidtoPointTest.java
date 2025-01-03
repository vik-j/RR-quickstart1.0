package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
public class PidtoPointTest extends LinearOpMode {
    public static Pose2d targetPose = new Pose2d(0,0,0);
    public static double epsilon = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Robot bot = new Robot(hardwareMap);
        Robot.p2p p2p = new Robot.p2p(bot, targetPose);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.epsilon = epsilon;
            bot.runPidToPoint(p2p, targetPose);
        }
    }
}
