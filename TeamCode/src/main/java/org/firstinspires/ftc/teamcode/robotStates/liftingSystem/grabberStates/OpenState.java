package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.grabberStates;

import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class OpenState extends RobotState<Grabber.StateType> {
    public OpenState() {
        super(Grabber.StateType.OPEN);
    }

    @Override
    public void execute() {
        // wouldn't need to do anything
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Grabber.StateType.OPENING;

    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return false;  // only switch states when gamePad input is taken, but that is tracked in liftingSystem class in its update functions
    }

    @Override
    public Grabber.StateType getNextStateType() {
        return Grabber.StateType.CLOSING;
    }
}
