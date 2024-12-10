package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class CollectTempState extends RobotState<Collector.StateType> {

    public CollectTempState() {
        super(Collector.StateType.COLLECTING_TEMP);
    }

    @Override
    public void execute() {
        // collect
        robot.getCollector().setSpindleMotorPower(Collector.COLLECT_TEMP_POWER);
    }
    @Override
    public boolean canEnter() {
        return robot.canCollect();
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return framesRunning > 1;
    }

    @Override
    public Collector.StateType getNextStateType() {
        return Collector.StateType.NOTHING;
    }
}
