package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class SearchingState extends RobotState<CollectingSystem.StateType> {

    public SearchingState() {
        super(CollectingSystem.StateType.SEARCH);
    }
    @Override
    public void execute() {
        StateManager<Extension.StateType> extensionManager = robot.getExtension().getStateManager();
        StateManager<Hinge.StateType> hingeManager = robot.getHinge().getStateManager();
        StateManager<Collector.StateType> collectorManager = robot.getCollector().getStateManager();

        // make the extension go to min position
        if (isFirstTime()) {
            extensionManager.tryEnterState(Extension.StateType.JUMP_TO_MIN);

            // if previous state was search and collect, this will hinge up and stop collector
            robot.getHinge().getTransitionState().setGoalState(Hinge.HINGE_UP_POSITION, Hinge.StateType.UP);
            collectorManager.tryEnterState(Collector.StateType.NOTHING);
        }

        // transitioning between doing nothing and spitting
        if (robot.getCollector().getStateManager().getActiveStateType() == Collector.StateType.NOTHING
                && robot.getHinge().getStateManager().getActiveStateType() != Hinge.StateType.UP)
            robot.getHinge().getTransitionState().setGoalState(Hinge.HINGE_UP_POSITION, Hinge.StateType.UP);
        else if (robot.getCollector().isSpitting() && robot.getHinge().getStateManager().getActiveStateType() != Hinge.StateType.MIDDLE)
            robot.getHinge().getTransitionState().setGoalState(Hinge.HINGE_MIDDLE_POSITION, Hinge.StateType.MIDDLE);
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
        return CollectingSystem.StateType.SEARCH;
    }
}
