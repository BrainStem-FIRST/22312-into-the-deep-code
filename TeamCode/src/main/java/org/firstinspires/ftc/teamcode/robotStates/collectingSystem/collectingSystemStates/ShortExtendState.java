package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class ShortExtendState extends RobotState<CollectingSystem.StateType> {

    public ShortExtendState() {
        super(CollectingSystem.StateType.SEARCH);
    }
    @Override
    public void execute() {
        robot.getExtension().setTargetPower(Extension.SEARCH_POWER);
        robot.getExtension().getStateManager().tryEnterState(Extension.StateType.FINDING_BLOCK);
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
        return robot.getExtension().getExtensionMotor().getCurrentPosition() >= Extension.SHORT_EXTEND_POSITION;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.SEARCH_AND_COLLECT;
    }
}
