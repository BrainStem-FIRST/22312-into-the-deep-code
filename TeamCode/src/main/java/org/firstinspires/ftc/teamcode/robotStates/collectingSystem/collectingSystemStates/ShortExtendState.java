package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class ShortExtendState extends RobotState<CollectingSystem.StateType> {

    public ShortExtendState() {
        super(CollectingSystem.StateType.SHORT_EXTEND);
    }
    @Override
    public void executeOnEntered() {
        robot.getExtension().getStateManager().tryEnterState(Extension.StateType.JUMP_TO_MIN);
        robot.setCanTransfer(false);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == CollectingSystem.StateType.IN;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.FINDING_BLOCK;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.SEARCH_AND_COLLECT;
    }
}
