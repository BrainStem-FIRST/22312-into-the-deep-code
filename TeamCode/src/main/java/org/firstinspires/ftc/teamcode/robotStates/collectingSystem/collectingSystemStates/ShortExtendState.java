package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class ShortExtendState extends RobotState<CollectingSystem.StateType> {

    public ShortExtendState() {
        super(CollectingSystem.StateType.SHORT_EXTEND);
    }
    @Override
    public void execute() {
        if (isFirstTime()) {
            robot.getExtension().getStateManager().tryEnterState(Extension.StateType.FINDING_BLOCK);
            robot.getExtension().setTargetPower(Extension.SEARCH_POWER);
        }
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
        if (isDone) {
            robot.getExtension().setTargetPower(0);
            robot.getExtension().setExtensionMotorPower(0);
            return true;
        }
        return false;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.SEARCH_AND_COLLECT;
    }
}
