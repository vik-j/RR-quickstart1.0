package org.firstinspires.ftc.teamcode.auton.limelightStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.Robot;

@Autonomous
public class LimelightAngleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        bot.limelightStart();
        bot.limelight.pipelineSwitch(2);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Angle", Math.toDegrees(bot.getSampleAngle()));
            telemetry.update();
        }
    }
}
