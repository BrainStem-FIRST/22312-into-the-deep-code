package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Grabber extends Subsystem {
    private enum State {
        COLLECTING, SPITTING, OFF
    }
    private State state;

    public Grabber(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        state = State.OFF;
    }
    public State getState() {
        return state;
    }
    public void setState(State state) {
        this.state = state;
    }
}
