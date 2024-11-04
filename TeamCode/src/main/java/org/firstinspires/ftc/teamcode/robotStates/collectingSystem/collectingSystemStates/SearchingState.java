package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class SearchingState extends RobotState<CollectingSystem.StateType> {

    public SearchingState() {
        super(CollectingSystem.StateType.SEARCH);
    }
    @Override
    public void execute() {
        robot.getExtension().getStateManager().tryEnterState(Extension.StateType.FINDING_BLOCK);
        robot.getCollector().getStateManager().tryEnterState(Collector.StateType.READY_TO_HINGE_DOWN);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == CollectingSystem.StateType.IN ||
                stateManager.getActiveStateType() == CollectingSystem.StateType.SEARCH_AND_COLLECT;
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
    public CollectingSystem.StateType getNextStateType() {
        return null;
    }
}
