package org.firstinspires.ftc.teamcode.stateMachine;
public interface State<StateType extends Enum<StateType>> {
    default void execute(double dt) {};
    default void executeOnEntered() {}
    default void executeOnExited() {}
    boolean canEnter();
    boolean canBeOverridden();
    boolean isDone();
    StateType getNextStateType();
}
