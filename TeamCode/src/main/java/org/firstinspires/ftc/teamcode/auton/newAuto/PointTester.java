package org.firstinspires.ftc.teamcode.auton.newAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class PointTester extends LinearOpMode {
    public Pose2d pose = new Pose2d(15, -62, Math.toRadians(270));
    public static double x = 15;
    public static double y = -62;
    public static double h = 270;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d target = new Pose2d(x,y, Math.toRadians(h));
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);
        Pose2d position = drive.pose;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            target = new Pose2d(x,y, Math.toRadians(h));

            Actions.runBlocking(drive.actionBuilder(position)
                    .strafeToLinearHeading(target.position, target.heading)
                    .build());

            position = drive.pose;
        }
    }
}
