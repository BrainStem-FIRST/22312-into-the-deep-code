package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.KnockBlockState;

public class InState extends RobotState<CollectingSystem.StateType> {

    public InState() {
        super(CollectingSystem.StateType.IN);
    }

    @Override
    public void executeOnEntered() {
        robot.getCollector().getStateManager().tryEnterState(Collector.StateType.NOTHING);
    }
    @Override
    public void execute(double dt) {
        robot.getExtension().getStateManager().tryEnterState(Extension.StateType.IN);

        if (robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.KNOCK_BLOCK)
            robot.getCollector().getStateManager().tryEnterState(Collector.StateType.COLLECTING_TEMP);
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
