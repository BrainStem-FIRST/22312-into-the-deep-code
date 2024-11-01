package org.firstinspires.ftc.teamcode.state;

public interface State<S extends Enum<S>> {
    void execute();
    boolean canEnter();
    boolean isDone();
    S getNextStateType(); // Change return type to S
    boolean canBeOverridden();
}
