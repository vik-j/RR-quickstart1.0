package org.firstinspires.ftc.teamcode.auton.limelightStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Autonomous
@Config
public class LimelightCorrection extends LinearOpMode {
    Limelight3A limelight;

    public static String targetColor = "blocksYellow";

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        telemetry.setMsTransmissionInterval(11);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        limelight.pipelineSwitch(0);

        String targetLabel1 = "blocksYellow";
        String targetLabel2 = "blocksBlue";
        String targetLabel3 = "blocksRed";

        String detectorResults;

        waitForStart();
        limelight.start();

        while (opModeIsActive() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();

            double tx = result.getTx();
            double ty = result.getTy();

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.update();

            List<DetectorResult> results = result.getDetectorResults();

            for (DetectorResult sense: results) {
                telemetry.addData("xPixels", sense.getTargetXPixels());
                telemetry.addData("yPixels", sense.getTargetYPixels());
                telemetry.update();
            }



            telemetry.addData("TempC", (limelight.getStatus().getTemp()));
            telemetry.addData("TempF", ((limelight.getStatus().getTemp()* 1.8)) + 32);
            telemetry.update();

        }

    }
}
