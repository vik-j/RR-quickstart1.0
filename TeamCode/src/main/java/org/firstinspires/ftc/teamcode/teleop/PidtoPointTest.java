package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class PidtoPointTest extends LinearOpMode {
    public static double targetX = 0, targetY = 0, targetH = 0;
    public Pose2d targetPose = new Pose2d(targetX,targetY,targetH);
    public static PIDCoefficients x = new PIDCoefficients(0.08,0,0);
    public static PIDCoefficients y = new PIDCoefficients(0.08,0,0);
    public static PIDCoefficients h = new PIDCoefficients(0.01,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Robot bot = new Robot(hardwareMap);
        PIDController xPID = new PIDController(x.p, x.i, x.d);
        PIDController yPID = new PIDController(y.p, y.i, y.d);
        PIDController hPID = new PIDController(h.p, h.i, h.d);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            targetPose = new Pose2d(targetX,targetY,targetH);

            xPID.setPID(x.p, x.i, x.d);
            yPID.setPID(y.p, y.i, y.d);
            hPID.setPID(h.p, h.i, h.d);

            double heading = drive.pose.heading.toDouble();

            double x = xPID.calculate(drive.pose.position.x, targetPose.position.x);
            double y = yPID.calculate(drive.pose.position.y, targetPose.position.y);
            double h = hPID.calculate(AngleUnit.normalizeRadians(heading), AngleUnit.normalizeRadians(targetPose.heading.toDouble()));

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("h", h);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(x,y), h));

//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);
//            double leftFrontPower = (x + y + h) / denominator;
//            double leftBackPower = (x - y + h) / denominator;
//            double rightFrontPower = (x - y - h) / denominator;
//            double rightBackPower = (x + y - h) / denominator;
//
//            drive.leftFront.setPower(leftFrontPower);
//            drive.leftBack.setPower(leftBackPower);
//            drive.rightFront.setPower(rightFrontPower);
//            drive.rightBack.setPower(rightBackPower);

//            double xRot = x*Math.cos(heading) - y*Math.sin(heading);
//            double yRot = x*Math.sin(heading) + y*Math.cos(heading);
//
//            telemetry.addData("xRot", xRot);
//            telemetry.addData("yRot", yRot);

//            drive.setPowers(xRot + yRot + h, xRot - yRot + h, xRot - yRot - h, xRot + yRot - h);
            telemetry.update();

            drive.updatePoseEstimate();

            telemetry.addData("driveX", drive.pose.position.x);
            telemetry.addData("driveY", drive.pose.position.y);
            telemetry.addData("driveH", drive.pose.heading.toDouble());
        }
    }
}
