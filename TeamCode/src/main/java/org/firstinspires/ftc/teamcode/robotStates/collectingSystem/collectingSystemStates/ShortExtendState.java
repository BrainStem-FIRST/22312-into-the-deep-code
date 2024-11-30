package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class ShortExtendState extends RobotState<CollectingSystem.StateType> {

    public ShortExtendState() {
        super(CollectingSystem.StateType.SHORT_EXTEND);
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
        boolean isDone = robot.getExtension().getExtensionMotor().getCurrentPosition() >= Extension.SHORT_EXTEND_POSITION;
        if (isDone)
            robot.getExtension().setTargetPower(0);
        return isDone;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.SEARCH_AND_COLLECT;
    }
}
