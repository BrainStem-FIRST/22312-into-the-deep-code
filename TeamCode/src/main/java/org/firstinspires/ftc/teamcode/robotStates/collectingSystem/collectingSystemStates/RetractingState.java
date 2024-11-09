package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;

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
        //robot.getCollector().getStateManager().tryEnterState(Collector.StateType.HINGE_UP);

        //wait for hinging to finish before retracting extension
        //if (robot.getCollector().getStateManager().getActiveStateType() == Collector.StateType.READY_TO_HINGE_DOWN ||
        //        robot.getCollector().getStateManager().getActiveStateType() == Collector.StateType.DONE_HINGING_UP)
        //    robot.getExtension().getStateManager().tryEnterState(Extension.StateType.RETRACTING);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == CollectingSystem.StateType.SEARCH ||
                stateManager.getActiveStateType() == CollectingSystem.StateType.SEARCH_AND_COLLECT;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.IN;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.IN;
    }
}
