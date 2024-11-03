package org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates;
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
        robot.getCollector().getStateManager().tryEnterState(Collector.StateType.NOTHING);
        robot.getExtension().getStateManager().tryEnterState(Extension.StateType.EXTENDING);
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
        return robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.OUT &&
                robot.getCollector().getStateManager().getActiveStateType() == Collector.StateType.COLLECTING;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.OUT;
    }
}
