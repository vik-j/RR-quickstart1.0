package org.firstinspires.ftc.teamcode.auton.qolActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

    public Action firstSpeci() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.newSpeci();
                return false;
            }
        };
    }
    public Action firstSpeci2() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.newSpeci2();
                return false;
            }
        };
    }
    public Action reset() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.reset();
                return false;
            }
        };
    }
    public Action autoSamplePickup() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.autoSamplePickup();
                return false;
            }
        };
    }
    public Action autoSampleSweeping() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.autoSampleSweeping();
                return false;
            }
        };
    }
    public Action specimenPickup() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.specimenPickup();
                return false;
            }
        };
    }
    public Action speciScoreReset() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.speciScoreReset();
                return false;
            }
        };
    }
    public Action specimenDeposit() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.specimenDeposit();
                return false;
            }
        };
    }
    public Action specimenDeposit2() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.specimenDeposit2();
                return false;
            }
        };
    }

    public Action grippyOpen() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.grippyOpen();
                return false;
            }
        };
    }
    public Action grippyClose() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.grippyClose();
                return false;
            }
        };
    }
    public Action sweepyUp() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.sweepyUp();
                return false;
            }
        };
    }
    public Action sweepyDown() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bot.sweepyDown();
                return false;
            }
        };
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
