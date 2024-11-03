package org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates;

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

    // waits till collector stops collecting/spitting and starts hinging up
    @Override
    public boolean isDone() {
        return robot.getCollector().getStateManager().getActiveStateType() == Collector.StateType.HINGE_UP;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.OUT;
    }
}
