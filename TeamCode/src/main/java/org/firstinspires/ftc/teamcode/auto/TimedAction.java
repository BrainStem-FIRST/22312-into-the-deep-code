package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimedAction implements Action {
    
    private ElapsedTime timer;

    public TimedAction() {
        timer = new ElapsedTime();
    }

    public double getTime() {
        return timer.seconds();
    }

    // this is meant to be overridden again when declaring it
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }
}
