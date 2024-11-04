package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.BlockColorSensor;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class CollectState extends RobotState<Collector.StateType> {

    public CollectState() {
        super(Collector.StateType.COLLECTING);
    }

    @Override
    public void execute() {
        robot.getCollector().setSpindleMotorPower(Collector.MAX_SPIN_POWER);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Collector.StateType.HINGE_DOWN ||
                stateManager.getActiveStateType() == Collector.StateType.SPITTING;
        // return true;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return time > 5;
        //return robot.getCollector().getColorSensor().getBlockColor() != BlockColorSensor.BlockColor.NONE;
    }

    @Override
    public Collector.StateType getNextStateType() {
        //if (robot.getCollector().hasValidBlockColor())
        //    return Collector.StateType.HINGE_UP;
        //return Collector.StateType.SPITTING;
        return Collector.StateType.HINGE_UP;
    }
}
