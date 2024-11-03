package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class SpitState extends RobotState<Collector.StateType> {

    public SpitState() {
        super(Collector.StateType.SPITTING);
    }

    @Override
    public void execute() {
        robot.getCollector().setSpindleMotorPower(-Collector.MAX_SPIN_POWER);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Collector.StateType.COLLECTING;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return time >= Collector.SPITTING_TIME;
    }

    @Override
    public Collector.StateType getNextStateType() {
        if (robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.OUT)
            return Collector.StateType.COLLECTING;
        return Collector.StateType.NOTHING;
    }
}
