package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.grabberStates;

import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class CloseState extends RobotState<Grabber.StateType> {
    public CloseState() {
        super(Grabber.StateType.CLOSED);
    }
    @Override
    public void execute() {}

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Grabber.StateType.CLOSING;
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
    public Grabber.StateType getNextStateType() {
        return Grabber.StateType.OPENING;
    }
}
