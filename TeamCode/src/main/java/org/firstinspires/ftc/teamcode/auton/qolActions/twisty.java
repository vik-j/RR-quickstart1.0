package org.firstinspires.ftc.teamcode.auton.qolActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.teleop.Robot;

public class twisty implements Action {
    Robot bot;
    double pos;
    public twisty(Robot bot, double pos) {
        this.bot = bot;
        this.pos = pos;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        bot.twisty.setPosition(bot.scaleTwisty(pos));
        return false;
    }
}
