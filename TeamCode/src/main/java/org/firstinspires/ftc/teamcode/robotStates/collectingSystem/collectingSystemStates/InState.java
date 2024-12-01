package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class InState extends RobotState<CollectingSystem.StateType> {

    public InState() {
        super(CollectingSystem.StateType.IN);
    }
    @Override
    public void execute() {
        if(isFirstTime())
            robot.getCollector().getStateManager().tryEnterState(Collector.StateType.NOTHING);
        robot.getExtension().getStateManager().tryEnterState(Extension.StateType.IN);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == CollectingSystem.StateType.RETRACTING;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public CollectingSystem.StateType getNextStateType() {
        return CollectingSystem.StateType.IN;
    }
}
