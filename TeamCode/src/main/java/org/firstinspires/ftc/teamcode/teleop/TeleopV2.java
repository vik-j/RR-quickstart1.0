package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "🐷")
public class TeleopV2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        bot.setTelemToDashboard(telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.arcadeDrive(gamepad1);
//            bot.slideControl(gamepad2);
//            bot.tiltControl(gamepad2);
           bot.TeleopPID(gamepad2);
           bot.clawControl(gamepad2);
           bot.twistyControl(gamepad2);
           bot.hangControl(gamepad2);
//            bot.slidesPID(gamepad2);
//            bot.wristControl(gamepad2);
//            bot.intakeControl(gamepad2);
            bot.scoringMacro(gamepad1, gamepad2);
//            bot.extraD1Features(gamepad1);

//            bot.updateAxonPositions();

            telemetry.addData("FlipPos", bot.flip.getCurrentPosition());
            telemetry.addData("SlidePos", bot.slide.getCurrentPosition());
//            telemetry.addData("wristPos", bot.wrist.getPosition());
            telemetry.addData("flipTarget", bot.armTarget);
            telemetry.addData("slideTarget", bot.slideTarget);
            telemetry.addData("fliPower", bot.flip.getPower());
            telemetry.addData("slidePower", bot.slide.getPower());
//            telemetry.addData("intakeLeft", bot.intakeLeft.getPower());
//            telemetry.addData("intakeRight", bot.intakeRight.getPower());
            telemetry.addData("degrees", bot.flip.getCurrentPosition() / (2048/90.0));
            telemetry.addData("limit", bot.slideExtensionLimit);
            telemetry.addData("Left Distance", bot.lookyLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Distance", bot.lookyRight.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
