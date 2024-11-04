package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class OutState extends RobotState<CollectingSystem.StateType> {

    public OutState() {
        super(CollectingSystem.StateType.OUT);
    }
    @Override
    public void execute() {
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == CollectingSystem.StateType.EXTENDING;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    // should only exit by overriding
    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.RETRACTING;
    }
}
