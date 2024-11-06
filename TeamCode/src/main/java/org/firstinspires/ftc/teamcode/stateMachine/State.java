package org.firstinspires.ftc.teamcode.stateMachine;
public interface State<StateType extends Enum<StateType>> {
    void execute();
    // are all of the safety requirements met so that mechanically, this state is able to be entered
    boolean canEnter();
    boolean canBeOverridden();
    boolean isDone();
    StateType getNextStateType();
}
