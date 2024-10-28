package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Grabber extends Subsystem {
    private final ServoImplEx grabServo;

    public enum State {
        CLOSED, OPEN
    }
    HashMap<State, Double> statePositions = new HashMap<>();
    private State state = State.CLOSED;
    private State goalState = null;

    public Grabber(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);
        grabServo = hwMap.get(ServoImplEx.class, "LiftGrabServo");
        initializeStatePositions();
    }

    // TODO: find actual state positions of grabber
    private void initializeStatePositions() {
        statePositions.put(State.CLOSED, 0.0);
        statePositions.put(State.OPEN, 1.0);
    }

    public ServoImplEx getGrabServo() {
        return grabServo;
    }
    public State getGoalState() {
        return goalState;
    }
    public void setGoalState(State goalState) {
        this.goalState = goalState;
    }
    public State getState() {
        return state;
    }
    public void setState(State state) {
        this.state = state;
    }
    public HashMap<State, Double> getStatePositions() {
        return statePositions;
    }
}
