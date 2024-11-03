package org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class RetractingState extends RobotState<CollectingSystem.StateType> {

    public RetractingState() {
        super(CollectingSystem.StateType.RETRACTING);
    }
    @Override
    public void execute() {
        robot.getExtension().getStateManager().tryEnterState(Extension.StateType.RETRACTING);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == CollectingSystem.StateType.OUT;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.IN &&
                robot.getCollector().getStateManager().getActiveStateType() == Collector.StateType.NOTHING;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.IN;
    }
}

