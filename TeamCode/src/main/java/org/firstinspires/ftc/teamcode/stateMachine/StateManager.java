package org.firstinspires.ftc.teamcode.stateMachine;

import java.util.HashMap;
import java.util.Objects;

public class StateManager<StateType extends Enum<StateType>> {

    private HashMap<StateType, BaseState<StateType>> stateMap;
    private StateType defaultStateType;
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
    public void setupStates(Object...args) {
        for (BaseState<StateType> state : stateMap.values()) {
            state.setup(args);
        }
    }

    public BaseState<StateType> getActiveState() {
        return stateMap.get(activeStateType);
    }
    public StateType getActiveStateType() {
        return activeStateType;
    }

    // sole function used for switching between states
    public boolean tryEnterState(StateType stateType) {
        System.out.println("try enter " + stateType);
        // only don't allow transition if active state is not done and cannot be overridden
        if (Objects.requireNonNull(stateMap.get(stateType)).canEnter() && (getActiveState().isDone() || getActiveState().canBeOverridden())) {
            System.out.println("entering");
            getActiveState().resetTime();
            activeStateType = stateType;
            return true;
        }
        System.out.println("could not enter");
        return false;
    }

    public void update(double dt) {
        getActiveState().execute();
        getActiveState().incrementTime(dt);

        if (getActiveState().isDone()) {
            System.out.println(getActiveState() + " is done");
            boolean canEnterState = tryEnterState(getActiveState().getNextStateType());
            if (!canEnterState) tryEnterState(defaultStateType);
        }
    }
}

/*
class thing {

    public static void main() {
        stateManager.add(StateType.RUN, new State() {
            @Override
            public void execute(LiftingSystem liftingSystem) {
                liftingSystem.getLift().raiseTo(position);
                if (liftingSysetm.getLift().position() > threshold) {
                    liftingSystem.getArm().rotateTo(position)
                }
            }
            @Override
            public void canEnter(LiftingSystem liftingSystem) {
                return liftingSystem.getArm.getPosition() == position
            }
            @Override
            public boolean canBeOverridden() {
                return true;
            }
            @Override
            public void isDone(StateManager stateManager, LiftingSystem liftingSystem) {
                return stateManager.getActiveStateTime() >= 1;
            }
            @Override
            public StateType getNextStateType() {
                return StateType.NOTHING;
            }
        });
    }
}

if (gamepad.a) {
    if (stateManager.getActiveState() == nothing)
        liftingSystem.getStateManager.tryEnterState(precSpecCollect);
    else if (stateManager.getActiveState() == waitToCollect)
        liftingSystem.getStateManager.tryEnterState(collect);
}

*/

