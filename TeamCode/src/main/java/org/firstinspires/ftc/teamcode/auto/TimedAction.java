package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimedAction implements Action {
    
    private ElapsedTime timer;
    private int framesRunning;

    public TimedAction() {
        timer = new ElapsedTime();
        framesRunning = 0;
    }

    public double getTime() {
        return timer.seconds();
    }

    public int getFramesRunning() {
        return framesRunning;
    }

    public void updateFramesRunning() {
        framesRunning++;
    }

    // this is meant to be overridden again when declaring it
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }
}
