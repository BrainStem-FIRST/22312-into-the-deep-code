package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class RetractingState extends RobotState<CollectingSystem.StateType> {
    public RetractingState() {
        super(CollectingSystem.StateType.RETRACTING);
    }
    @Override
    public void execute() {
        // force hinge to be in up position
        if (robot.getHinge().getStateManager().getActiveStateType() != Hinge.StateType.UP)
            robot.getHinge().getTransitionState().overrideGoalState(Hinge.HINGE_UP_POSITION, Hinge.StateType.UP, Hinge.HINGE_UP_TIME);

        // retract when hinging is done
        if (robot.getHinge().getStateManager().getActiveStateType() == Hinge.StateType.UP)
                robot.getExtension().getStateManager().tryEnterState(Extension.StateType.RETRACTING);

        // collect while retracting to make sure block stays in
        robot.getCollector().getStateManager().tryEnterState(Collector.StateType.COLLECTING_TEMP);
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
