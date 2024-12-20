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
    public void executeOnEntered() {
        robot.getExtension().getStateManager().tryEnterState(Extension.StateType.FINDING_BLOCK);
        robot.getHinge().getTransitionState().setGoalState(Hinge.HINGE_DOWN_POSITION, Hinge.StateType.DOWN);
    }
    @Override
    public void execute(double dt) {

        // go to search mode
        if (robot.getInput().getGamepadTracker1().isRightTriggerPressed())
            robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.SEARCH);

        // collect once hinging is finished
        // nothing is default state - only collect then b/c do not want to override collect/spit temp
        if (robot.getCollector().getStateManager().getActiveStateType() == Collector.StateType.NOTHING)
            robot.getCollector().getStateManager().tryEnterState(Collector.StateType.COLLECTING);

        // collector determines state of hinge
        // transitioning between collecting and spitting hinge states
        if (robot.getCollector().isCollecting())
            robot.getHinge().goToHingeDownState();
        else if (robot.getCollector().isSpitting())
            robot.getHinge().goToHingeMiddleState();

        // left trigger retracts
        if (robot.getInput().getGamepadTracker1().isFirstFrameLeftTrigger())
            robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.RETRACTING);
    }

    @Override
    public boolean canEnter() {
        return (stateManager.getActiveStateType() == CollectingSystem.StateType.SEARCH || stateManager.getActiveStateType() == CollectingSystem.StateType.SHORT_EXTEND)
                && robot.getExtension().getExtensionMotorPosition() >= Extension.MIN_SEARCH_AND_COLLECT_POSITION;
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
