package org.firstinspires.ftc.teamcode.stateMachine;

import androidx.annotation.NonNull;

public abstract class BaseState<StateType extends Enum<StateType>> implements State<StateType> {

    protected StateType stateType;
    protected StateManager<StateType> stateManager;
    protected double time;

    public BaseState(StateType stateType) {
        this.stateType = stateType;
    }

    @NonNull
    @Override
    public String toString() {
        return "BaseState(" + stateType + " | " + "time:" + time + ")";
    }

    public void resetTime() {
        time = 0;
    }
    public void incrementTime(double dt) {
        time += dt;
    }
    public boolean isFirstTime() {
        return time == 1;
    }

    public abstract void setup(Object...args);
}
