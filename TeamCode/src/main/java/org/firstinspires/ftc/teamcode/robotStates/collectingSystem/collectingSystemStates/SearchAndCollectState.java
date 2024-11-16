package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class SearchAndCollectState extends RobotState<CollectingSystem.StateType> {

    public SearchAndCollectState() {
        super(CollectingSystem.StateType.SEARCH_AND_COLLECT);
    }
    @Override
    public void execute() {
        if (isFirstTime()) {
            robot.getExtension().getStateManager().tryEnterState(Extension.StateType.FINDING_BLOCK);
            robot.getHinge().getStateManager().tryEnterState(Hinge.StateType.HINGING_DOWN);
        }
        // collect once hinging is finished
        if (robot.getHinge().getStateManager().getActiveStateType() == Hinge.StateType.DOWN)
            robot.getCollector().getStateManager().tryEnterState(Collector.StateType.COLLECTING);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == CollectingSystem.StateType.SEARCH;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return robot.getCollector().getStateManager().getActiveStateType() == Collector.StateType.VALID_BLOCK;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.RETRACTING;
    }
}
