package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class PidtoPointTest extends LinearOpMode {
    public static Pose2d targetPose = new Pose2d(0,0,0);
    public static PIDCoefficients x = new PIDCoefficients(0,0,0);
    public static PIDCoefficients y = new PIDCoefficients(0,0,0);
    public static PIDCoefficients h = new PIDCoefficients(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Robot bot = new Robot(hardwareMap);
        PIDController xPID = new PIDController(x.p, x.i, x.d);
        PIDController yPID = new PIDController(y.p, y.i, y.d);
        Robot.HeadingPIDController hPID = new Robot.HeadingPIDController(h);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            xPID.setPID(x.p, x.i, x.d);
            yPID.setPID(y.p, y.i, y.d);
            hPID.setPID(h);

            double heading = drive.pose.heading.toDouble();

            double x = xPID.calculate(drive.pose.position.x, targetPose.position.x);
            double y = yPID.calculate(drive.pose.position.y, targetPose.position.y);
            double h = hPID.calculate(heading, targetPose.heading.toDouble());

            double xRot = x*Math.cos(heading) - y*Math.sin(heading);
            double yRot = x*Math.sin(heading) + y*Math.cos(heading);

            drive.setPowers(xRot + yRot + h, xRot - yRot + h, xRot - yRot - h, xRot + yRot - h);
        }
    }
}
