package org.firstinspires.ftc.teamcode.stateMachine;
public interface State<StateType extends Enum<StateType>> {
    public void execute();
    // are all of the safety requirements met so that mechanically, this state is able to be entered
    public boolean canEnter();
    public boolean canBeOverridden();
    public boolean isDone();
    public StateType getNextStateType();
}
