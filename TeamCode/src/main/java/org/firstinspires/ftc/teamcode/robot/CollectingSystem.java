package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.state.State;
import org.firstinspires.ftc.teamcode.state.StateManager;

import java.util.ArrayList;
import java.util.Arrays;

public class CollectingSystem {

    public enum StateType {
        NOTHING, IN, EXTENDING, COLLECTING, RETRACTING
    }
    private final StateManager<StateType> stateManager;
    public CollectingSystem() {
        stateManager = new StateManager<>(StateType.class);

        stateManager.addState(
            StateType.IN,
            new State<StateType>() {
                @Override
                public void execute() {

                }
                @Override
                public boolean canEnter() {
                    return false;
                }
                @Override
                public boolean isDone() {
                    return false;
                }
                @Override
                public StateType getNextStateType() {
                    return StateType.NOTHING;
                }
                @Override
                public boolean canBeOverridden() {
                    return false;
                }
            }
        );
    }
    public void tryExtendAndCollect() {
        stateManager.tryEnter(StateType.EXTENDING);
    }
}
