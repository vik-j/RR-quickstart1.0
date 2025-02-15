package org.firstinspires.ftc.teamcode.auton.limelightStuff;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@TeleOp
public class LimeLightTest extends LinearOpMode {
    public Limelight3A cam;
    MecanumDrive drive;
    Robot bot;
    public double armTarget = 0, slideTarget = 0;
//    public static double wristPos = 1;

    //TODO: optimal distance is 3.5 inches away
    //TODO: limelight is 6.5 inches offset horizontally
    //TODO: limelight is offset 10.5 inches from inside edge of sub support
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        bot = new Robot(hardwareMap);

        waitForStart();


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

                Vector2d samplePos = Robot.getDistFromCamera(xPixels, yPixels);

                telemetry.addData("sampleX", samplePos.x);
                telemetry.addData("sampleY", samplePos.y);

                if (gamepad1.b) {
                    drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
                    Actions.runBlocking(
                            new ParallelAction(
                            drive.actionBuilder(new Pose2d(0,0,0))
                                    .afterTime(0, telemetryPacket -> {
                                        slideTarget = Range.clip(Robot.slidesTTS*(samplePos.y), 0, 1830);
                                        bot.slideTargetAuto = (int) slideTarget;
                                        return false;
                                    })
                                    .strafeToConstantHeading(new Vector2d(0, -samplePos.x - 6.5))
                                    .afterTime(0.5, telemetryPacket -> {
                                        bot.flippy.setPosition(0.4);
                                        return false;
                                    })
                                    .afterTime(1.5, telemetryPacket -> {
                                        bot.grippyClose();
                                        return false;
                                    })
                                    .build(),
                            bot.getPIDAction()));
                }
            }
            else {
                telemetry.addLine("Nothing is There");
            }
            double flipPos = bot.flip.getCurrentPosition();
            double slidePos = bot.slide.getCurrentPosition();

            double pid = bot.armController.calculate(flipPos, armTarget);
            double ff = Math.cos(Math.toRadians(armTarget / Robot.armPIDValues.ticks_in_degree)) * Robot.armPIDValues.fF;

            double power = pid + ff;

            bot.flip.setPower(power);

            double pid2 = bot.slideController.calculate(slidePos, bot.scaleSlides(slideTarget));

            bot.slide.setPower(pid2);
            telemetry.update();

//            bot.wrist.setPosition(wristPos);

        }
    }
}
//Hello World!