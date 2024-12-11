package org.firstinspires.ftc.teamcode.stateMachine;

import java.util.HashMap;
import java.util.Objects;

public class StateManager<StateType extends Enum<StateType>> {

    private final HashMap<StateType, BaseState<StateType>> stateMap;
    private final StateType defaultStateType;
    private StateType activeStateType;

    public StateManager(StateType defaultStateType) {
        this.defaultStateType = defaultStateType;
        this.activeStateType = defaultStateType;
        this.stateMap = new HashMap<>();
    }

    // forces state and state manager to have same enum
    public void addState(StateType stateType, BaseState<StateType> state) {
        stateMap.put(stateType, state);
    }

    // have to call this function before running any state logic
    // passes information given to states (used deprecation bc info needed might vary from one state manager to another)
    public void setupStates(Object...args) {
        for (BaseState<StateType> state : stateMap.values()) {
            state.setup(args);
        }
    }

    public BaseState<StateType> getState(StateType stateType) {
        return stateMap.get(stateType);
    }
    public BaseState<StateType> getActiveState() {
        return stateMap.get(activeStateType);
    }
    public StateType getActiveStateType() {
        return activeStateType;
    }

    // sole function used for switching between states
    public boolean tryEnterState(StateType stateType) {
        // only allow entering if meet requirements to enter state and either current state is done or current state can be overridden
        if (Objects.requireNonNull(stateMap.get(stateType)).canEnter() && (getActiveState().isDone() || getActiveState().canBeOverridden())) {
            // resetting previous state
            getActiveState().resetTime();
            getActiveState().resetFramesRunning();
            getActiveState().executeOnExited();

            // setting up for next state
            activeStateType = stateType;
            getActiveState().resetTime();
            getActiveState().resetFramesRunning();
            getActiveState().executeOnEntered();
            return true;
        }
        return false;
    }

    public void update(double dt) {
        if (getActiveState().isDone()) {
            tryEnterState(getActiveState().getNextStateType());
        }

        getActiveState().execute(dt);
        getActiveState().incrementTime(dt);
        getActiveState().incrementFramesRunning();
    }
}