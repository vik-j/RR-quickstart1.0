package org.firstinspires.ftc.teamcode.auton.qolActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.teleop.Robot;

public class arm implements Action {
    Robot bot;
    int flipPos, slidePos;

    public arm(Robot bot, int flipPos, int slidePos) {
        this.bot = bot;
        this.flipPos = flipPos;
        this.slidePos = slidePos;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        bot.setPidValues(flipPos, slidePos);
        return false;
    }
}
