package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class HangTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        bot.resetEncoders();
        bot.setTelemToDashboard(telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad2.a) {
                bot.hang1();
            }
            else if (gamepad2.b) {
                bot.hang2();
            }
            else if (gamepad2.x) {
                bot.hang3();
            }
            else if (gamepad2.y) {
                bot.hang4();
            }
            else if (gamepad2.dpad_up) {
                bot.hang5();
            }
            else if (gamepad2.dpad_down) {
                bot.hang6();
            }
            bot.arcadeDrive(gamepad1);
            bot.TeleopPID(gamepad2);
        }

    }
}
