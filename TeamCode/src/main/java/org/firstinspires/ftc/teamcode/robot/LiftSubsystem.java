package org.firstinspires.ftc.teamcode.robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

// created parent class for lift subsystems because all share same state properties
public abstract class LiftSubsystem extends Subsystem {

    // assuming each state represents being "ready", not actually executing the potential action associated with it
    public enum State {
        TROUGH, RAM, BASKET, BACK_DROP
    }
    private static final HashMap<State, Double> prepStatePositions = new HashMap<>();
    private static final HashMap<State, Double> execStatePositions = new HashMap<>();
    private State curState = State.TROUGH;
    private State goalState = null; // if null, then not in transition (ready to execute)
    public LiftSubsystem(HardwareMap hw, Telemetry telemetry, AllianceColor allianceColor) {
        super(hw, telemetry, allianceColor);
    }
    public void setPrepStatePositions(double trough, double ram, double basket, double backdrop) {
        prepStatePositions.put(State.TROUGH, trough);
        prepStatePositions.put(State.RAM, ram);
        prepStatePositions.put(State.BASKET, basket);
        prepStatePositions.put(State.BACK_DROP, backdrop);
    }
    public void setExecStatePositions(double trough, double ram, double basket, double backdrop) {
        execStatePositions.put(State.TROUGH, trough);
        execStatePositions.put(State.RAM, ram);
        execStatePositions.put(State.BASKET, basket);
        execStatePositions.put(State.BACK_DROP, backdrop);
    }

    public void goalStateReached() {
        curState = goalState;
        goalState = null;
    }
    public abstract boolean executeCurrentState();

    // getters/setters
    public State getGoalState() {
        return goalState;
    }

    // returns if desired state is already set or not; going to use in liftingSubsystemTele class to perform checks on each system whilst simultaneously setting states
    public boolean setGoalState(State goalState) {
        if(goalState != curState)
            this.goalState = goalState;
        return goalState == curState;
    }
    public State getCurState() {
        return curState;
    }
    public void setCurState(State state) {
        this.curState = state;
    }

    public HashMap<State, Double> getExecStatePositions() {
        return execStatePositions;
    }
    public HashMap<State, Double> getPrepStatePositions() {
        return prepStatePositions;
    }
}
