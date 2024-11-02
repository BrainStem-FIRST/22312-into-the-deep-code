package org.firstinspires.ftc.teamcode.robotStates;

public class NothingState<StateType extends Enum<StateType>> extends RobotStateTele<StateType> {
    public NothingState(StateType stateType) {
        super(stateType);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean canEnter() {
        return false;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
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
