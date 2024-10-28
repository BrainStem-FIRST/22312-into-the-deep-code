package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class Arm extends Subsystem {
    private final ServoImplEx armServo;
    public enum State {
        DOWN, UP, AT_HARD_STOP, AT_PICKUP
    }

    // included hashmap because three positions, easier to deal with each case; NOTE: CAN KNOW WHEN IN TRANSITION WHEN GOAL STATE != CURRENT STATE (applicable for other subsystems with same logic)
    private static HashMap<State, Double> statePositions = new HashMap<>();
    private State state = State.DOWN;
    private State goalState = null;

    public Arm(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);
        armServo = hwMap.get(ServoImplEx.class, "LiftArmServo");
        initializeStatePositions();
    }

    // TODO: find actual state positions of arm
    private void initializeStatePositions() {
        statePositions.put(State.DOWN, 0.0);
        statePositions.put(State.UP, 0.5);
        statePositions.put(State.AT_HARD_STOP, 0.75);
        statePositions.put(State.AT_PICKUP, 0.25);
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

    public ServoImplEx getArmServo() {
        return armServo;
    }
}
