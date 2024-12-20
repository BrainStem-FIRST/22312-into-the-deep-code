package org.firstinspires.ftc.teamcode.stateMachine;
public interface State<StateType extends Enum<StateType>> {
    boolean canEnter();
    default void executeOnEntered() {}
    default void execute(double dt) {};
    default void executeOnExited() {}
    boolean canBeOverridden();
    boolean isDone();
    default StateType getNextStateType() { return null; };
}
