package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class RetractingState extends RobotState<CollectingSystem.StateType> {

    private boolean waitingForHinge;
    public RetractingState() {
        super(CollectingSystem.StateType.RETRACTING);
        waitingForHinge = false;
    }
    @Override
    public void execute() {

        if (isFirstTime()) {
            robot.getCollector().getStateManager().tryEnterState(Collector.StateType.NOTHING); // initially stop collector
            waitingForHinge = robot.getCollectingSystem().hingeMustBeUp(); // if the extension needs to wait for the hinge, just wait for it to go all the way up and then retract
        }

        // force hinge to be in up position
        if (robot.getHinge().getStateManager().getActiveStateType() != Hinge.StateType.UP)
            robot.getHinge().getTransitionState().setGoalState(Hinge.HINGE_UP_POSITION, Hinge.StateType.UP);

        // case for retract only when hinge is done
        if (waitingForHinge && robot.getHinge().getStateManager().getActiveStateType() == Hinge.StateType.UP)
                robot.getExtension().getStateManager().tryEnterState(Extension.StateType.RETRACTING);

        // case for retract immediately
        if (!waitingForHinge)
            robot.getExtension().getStateManager().tryEnterState(Extension.StateType.RETRACTING);
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
