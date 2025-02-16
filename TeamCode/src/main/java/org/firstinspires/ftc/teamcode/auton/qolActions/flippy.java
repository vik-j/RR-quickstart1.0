package org.firstinspires.ftc.teamcode.auton.qolActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.teleop.Robot;

public class flippy implements Action {
    Robot bot;
    double pos;
    public flippy(Robot bot, double pos) {
        this.bot = bot;
        this.pos = pos;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        bot.flippy.setPosition(pos);
        return false;
    }
}
