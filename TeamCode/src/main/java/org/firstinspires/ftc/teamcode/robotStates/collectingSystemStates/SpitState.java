package org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotStateTele;
import org.firstinspires.ftc.teamcode.tele.CollectingSystemTele;

public class SpitState extends CollectSystemState {

    public SpitState() {
        super(CollectingSystemTele.StateType.SPITTING);
    }

    @Override
    public void execute() {
        robot.getCollector().setSpindleMotorPower(-Collector.MAX_SPIN_POWER);
    }

    @Override
    public boolean canEnter() {
        return collectingSystemTele.getStateManager().getActiveStateType() == CollectingSystemTele.StateType.COLLECTING;
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
    public CollectingSystemTele.StateType getNextStateType() {
        return CollectingSystemTele.StateType.COLLECTING;
    }
}
