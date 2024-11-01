package org.firstinspires.ftc.teamcode.state;

import java.util.EnumMap;

public class StateManager<S extends Enum<S>> {
    private final EnumMap<S, State<S>> stateMap;
    private S activeStateType;

    public StateManager(Class<S> stateClass) {
        stateMap = new EnumMap<>(stateClass);
    }

    public void addState(S stateType, State<S> state) {
        stateMap.put(stateType, state);
    }

    public void tryEnter(S newStateType) {
        State<S> newState = getState(newStateType);
        State<S> activeState = getActiveState();

        if (newState.canEnter() && (activeState.isDone() || activeState.canBeOverridden())) {
            activeStateType = newStateType;
        }
    }

    public void update() {
        State<S> activeState = getActiveState();
        activeState.execute();
        if (activeState.isDone()) {
            tryEnter(activeState.getNextStateType());
        }
    }

    // Getters
    public State<S> getState(S stateType) {
        return stateMap.get(stateType);
    }

    public State<S> getActiveState() {
        return getState(activeStateType);
    }
}
