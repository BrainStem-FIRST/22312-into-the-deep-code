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
        // only want to keep trying to enter state if not already entered
        robot.getCollector().getStateManager().tryEnterState(Collector.StateType.HINGE_DOWN);
        robot.getExtension().getStateManager().tryEnterState(Extension.StateType.EXTENDING);
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
        robot.telemetry.addData("collector state: ", robot.getCollector().getStateManager().getActiveStateType());
        return robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.OUT &&
                robot.getCollector().getStateManager().getActiveStateType() == Collector.StateType.COLLECTING;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.OUT;
    }
}
