package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class CollectState extends RobotState<Collector.StateType> {
    public CollectState() {
        super(Collector.StateType.COLLECTING);
    }

    @Override
    public void execute(double dt) {
        // collect and handle block jamming
        if (robot.getCollector().getTeleCurrentTracker().hasValidatedAbnormalCurrent())
            robot.getCollector().setSpindleMotorPower(Collector.SPIT_TEMP_POWER);
        else
            robot.getCollector().setSpindleMotorPower(Collector.TELE_COLLECT_POWER);
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
        return robot.getCollector().hasValidBlockColor() ? Collector.StateType.VALID_BLOCK : Collector.StateType.SPITTING;
    }
}
