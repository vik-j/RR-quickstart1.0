package org.firstinspires.ftc.teamcode.auton.qolActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.teleop.Robot;

import java.util.ArrayList;

public class qol {
    Robot bot;

    public qol(Robot bot) {
        this.bot = bot;
    }

    public Action combine(Action... actions) {
        return new SequentialAction(actions);
    }

    public Action newAutoSpeci() {
        return new InstantAction(() -> bot.newAutoSpeci());
    }
    public Action samplePivot() {
        return new InstantAction(() -> bot.samplePivot());
    }
    public Action sampleSlides() {
        return new InstantAction(() -> bot.sampleSlides());
    }
    public Action firstSpeci() {
        return new InstantAction(() -> bot.newSpeci());
    }
    public Action firstSpeci2() {
        return new InstantAction(() -> bot.newSpeci2());
    }
    public Action reset() {
        return new InstantAction(() -> bot.reset());
    }
    public Action autoSamplePickup() {
        return new InstantAction(() -> bot.autoSamplePickup());
    }
    public Action autoSamplePickup2() {
        return new InstantAction(() -> bot.autoSamplePickup2());
    }
    public Action autoSampleSweeping() {
        return new InstantAction(() -> bot.autoSampleSweeping());
    }
    public Action specimenPickup() {
        return new InstantAction(() -> bot.specimenPickup());
    }
    public Action speciScoreReset() {
        return new InstantAction(() -> bot.speciScoreReset());
    }
    public Action specimenDeposit() {
        return new InstantAction(() -> bot.specimenDeposit());
    }
    public Action specimenDeposit2() {
        return new InstantAction(() -> bot.specimenDeposit2());
    }
    public Action hangAlmostDown() {
        return new InstantAction(() -> bot.hangAlmostDown());
    }
    public Action hangUp() {
        return new InstantAction(() -> bot.hangUp());
    }
    public Action hangDown() {
        return new InstantAction(() -> bot.hangDown());
    }

    public Action grippyOpen() {
        return new InstantAction(() -> bot.grippyOpen());
    }
    public Action grippyClose() {
        return new InstantAction(() -> bot.grippyClose());
    }
    public Action sweepyUp() {
        return new InstantAction(() -> bot.sweepyUp());
    }
    public Action sweepyDown() {
        return new InstantAction(() -> bot.sweepyDown());
    }
    public Action flippy(double pos) {
        return new flippy(bot, pos);
    }
    public Action twisty(double pos) {
        return new twisty(bot, pos);
    }
    public Action arm(int pivot, int slide) {
        return new arm(bot, pivot, slide);
    }
}
