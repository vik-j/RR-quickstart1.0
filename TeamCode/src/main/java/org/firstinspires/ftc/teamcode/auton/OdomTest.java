package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.Robot;

@Disabled
@Autonomous
public class OdomTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

//        while (opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("ForwardPodTicks", bot.leftBack.getCurrentPosition());
//            telemetry.addData("StrafePodTicks", bot.rightFront.getCurrentPosition());
//            telemetry.update();
//        }
    }
}
