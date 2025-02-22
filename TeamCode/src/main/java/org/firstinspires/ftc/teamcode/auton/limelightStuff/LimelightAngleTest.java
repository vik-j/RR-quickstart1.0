package org.firstinspires.ftc.teamcode.auton.limelightStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@TeleOp
public class LimelightAngleTest extends LinearOpMode {
    public static int pipeline = 2;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        bot.limelightStart();
        bot.limelight.pipelineSwitch(pipeline);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.limelight.pipelineSwitch(pipeline);
            LLResult result = bot.limelight.getLatestResult();

            if (result != null && result.isValid()) {
                if (result.getColorResults().get(0).getTargetCorners().size()  == 4) {
                    telemetry.addData("Angle", bot.convertToClawDegree(bot.getSampleAngle()));
                    if (gamepad1.b) {
                        bot.flippy.setPosition(bot.scaleFlippy(0.4));
                        bot.twisty.setPosition(bot.convertDegreesToTwisty(bot.convertToClawDegree(bot.getSampleAngle())));
                    }
                }
                telemetry.addData("corners", result.getColorResults().get(0).getTargetCorners());
            }
            bot.flippy.setPosition(bot.scaleFlippy(0.7));
            telemetry.update();
        }
    }
}
