package org.firstinspires.ftc.teamcode.auton.limelightStuff;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@TeleOp
public class LimeLightTest extends LinearOpMode {
    public Limelight3A cam;
    public static double strafeP = 0, strafeI = 0, strafeD = 0;
    public PIDController strafeController;
    public static double multiplier = 0;
    MecanumDrive drive;

    //TODO: optimal distance is 3.5 inches away
    //TODO: limelight is 6.5 inches offset horizontally
    //TODO: limelight is offset 10.5 inches from inside edge of sub support
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        strafeController = new PIDController(strafeP, strafeI, strafeD);

        cam = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        cam.pipelineSwitch(0);

        cam.start();

        while (opModeIsActive() && !isStopRequested()) {
            LLResult result = cam.getLatestResult();

            if (result != null && result.isValid()) {
                double xPixels = result.getDetectorResults().get(0).getTargetXPixels();
                double yPixels = result.getDetectorResults().get(0).getTargetYPixels();

                telemetry.addData("pixelsX", xPixels);
                telemetry.addData("pixelsY", yPixels);

                telemetry.addData("id", result.getDetectorResults().get(0).getClassName());

                Vector2d samplePos = Robot.convertPixelsToInches(xPixels, yPixels);

                telemetry.addData("sampleX", samplePos.x);
                telemetry.addData("sampleY", samplePos.y);

//                strafeController.setPID(strafeP, strafeI, strafeD);
//
//                double pid = strafeController.calculate(xPixels, 0);
//
//                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, multiplier*pid), 0));
            }
            else {
                telemetry.addLine("Nothing is There");
            }
            telemetry.update();

        }
    }
}
//Hello World!