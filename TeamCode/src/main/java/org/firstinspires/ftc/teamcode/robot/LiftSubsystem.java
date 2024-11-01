package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.HashMap;


// created parent class for lift subsystems because all share same state properties
public abstract class LiftSubsystem extends Subsystem {

    // assuming each state represents being "ready", not actually executing the potential action associated with it
    public enum State {
        TROUGH, BLOCK_DROP, SPECIMEN_PICKUP, SPECIMEN_RAM, BASKET_DROP
    }
    private static final HashMap<State, Double> prepStatePositions = new HashMap<>();
    private static final HashMap<State, Double> execStatePositions = new HashMap<>();
    private State curState = State.TROUGH;
    private State goalState = null; // if null, then not in transition (ready to execute)
    private boolean curStateExecuting = false;
    public final double DESTINATION_THRESHOLD;

    public LiftSubsystem(HardwareMap hw, Telemetry telemetry, AllianceColor allianceColor, double destination_threshold) {
        super(hw, telemetry, allianceColor);
        DESTINATION_THRESHOLD = destination_threshold;
    }
    public void setPrepStatePositions(double trough, double backdrop, double pickup, double basket, double ram) {
        prepStatePositions.put(State.TROUGH, trough);
        prepStatePositions.put(State.BLOCK_DROP, backdrop);
        prepStatePositions.put(State.SPECIMEN_PICKUP, pickup);
        prepStatePositions.put(State.BASKET_DROP, basket);
        prepStatePositions.put(State.SPECIMEN_RAM, ram);
    }
    public void setExecStatePositions(double trough, double backdrop, double pickup, double basket, double ram) {
        execStatePositions.put(State.TROUGH, trough);
        execStatePositions.put(State.BLOCK_DROP, backdrop);
        execStatePositions.put(State.SPECIMEN_PICKUP, pickup);
        execStatePositions.put(State.BASKET_DROP, basket);
        execStatePositions.put(State.SPECIMEN_RAM, ram);
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
    public void setGoalState(State goalState) {
        this.goalState = goalState;
    }
    public State getCurState() {
        return curState;
    }
    public void setCurState(State state) {
        this.curState = state;
    }
    public boolean getCurStateExecuting() {
        return curStateExecuting;
    }
    public void setCurStateExecuting(boolean curStateExecuting) {
        this.curStateExecuting = curStateExecuting;
    }
    public HashMap<State, Double> getExecStatePositions() {
        return execStatePositions;
    }
    public HashMap<State, Double> getPrepStatePositions() {
        return prepStatePositions;
    }
}
