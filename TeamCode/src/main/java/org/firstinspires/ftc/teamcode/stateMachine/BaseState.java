package org.firstinspires.ftc.teamcode.stateMachine;

import androidx.annotation.NonNull;

public abstract class BaseState<StateType extends Enum<StateType>> implements State<StateType> {

    protected StateType stateType;
    protected double time;
    protected int framesRunning;

    public BaseState(StateType stateType) {
        this.stateType = stateType;
        time = 0;
        framesRunning = 0;
    }

    @NonNull
    @Override
    public String toString() {
        return "BaseState(" + stateType + " | " + "time:" + time + ")";
    }

    public double getTime() {
        return time;
    }
    public void resetTime() {
        time = 0;
    }
    public void resetFramesRunning() { framesRunning = 0; }
    public void incrementTime(double dt) {
        time += dt;
    }
    public int getFramesRunning() {
        return framesRunning;
    }
    public void incrementFramesRunning() { framesRunning++; }
    public boolean isFirstTime() {
        return framesRunning == 1;
    }

    public abstract void setup(Object...args);
}
