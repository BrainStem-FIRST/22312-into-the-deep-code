package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class ExtendingState extends RobotState<CollectingSystem.StateType> {

    public ExtendingState() {
        super(CollectingSystem.StateType.EXTENDING);
    }
    @Override
    public void execute() {
        robot.getExtension().getStateManager().tryEnterState(Extension.StateType.EXTENDING);
        robot.getCollector().getStateManager().tryEnterState(Collector.StateType.READY_TO_HINGE_DOWN);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == CollectingSystem.StateType.IN;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.OUT;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.OUT;
    }
}
