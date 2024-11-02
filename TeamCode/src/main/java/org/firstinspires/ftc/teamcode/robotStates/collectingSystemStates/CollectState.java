package org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotStateTele;
import org.firstinspires.ftc.teamcode.tele.CollectingSystemTele;

public class CollectState extends CollectSystemState {

    public CollectState() {
        super(CollectingSystemTele.StateType.COLLECTING);
    }

    @Override
    public void execute() {
        robot.getCollector().setSpindleMotorPower(Collector.MAX_SPIN_POWER);
    }

    @Override
    public boolean canEnter() {
        return collectingSystemTele.getStateManager().getActiveStateType() != CollectingSystemTele.StateType.RETRACTING &&
                collectingSystemTele.getStateManager().getActiveStateType() != CollectingSystemTele.StateType.IN;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return robot.getCollector().getBlockColor() != Collector.BlockColor.NONE;
    }

    @Override
    public CollectingSystemTele.StateType getNextStateType() {
        if (robot.getCollector().hasValidBlockColor())
            return CollectingSystemTele.StateType.RETRACTING;
        return CollectingSystemTele.StateType.SPITTING;
    }
}
