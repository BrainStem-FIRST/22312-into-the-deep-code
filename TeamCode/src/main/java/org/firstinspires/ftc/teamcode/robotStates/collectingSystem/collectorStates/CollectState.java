package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class CollectState extends RobotState<Collector.StateType> {
    public CollectState() {

        super(Collector.StateType.COLLECTING);
    }

    @Override
    public void execute() {
        if (isFirstTime())
            robot.getCollector().resetJamTracking();

        if (robot.getCollector().isJammed())
            robot.getCollector().setSpindleMotorPower(Collector.SPIT_TEMP_POWER);
        else
            robot.getCollector().setSpindleMotorPower(Collector.COLLECT_POWER);
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
        return robot.getCollector().getBlockColorSensor().hasValidatedColor();
    }

    @Override
    public Collector.StateType getNextStateType() {
        if (robot.getCollector().hasValidBlockColor())
            return Collector.StateType.VALID_BLOCK;
        return Collector.StateType.SPITTING;
    }
}
