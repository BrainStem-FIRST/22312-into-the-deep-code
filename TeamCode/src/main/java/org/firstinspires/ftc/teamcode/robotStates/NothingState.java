package org.firstinspires.ftc.teamcode.robotStates;

public class NothingState<StateType extends Enum<StateType>> extends RobotState<StateType> {
    public NothingState(StateType stateType) {
        super(stateType);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean canEnter() {
        return true;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public StateType getNextStateType() {
        return null;
    }
}
