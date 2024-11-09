package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;
public class HingeUpState extends RobotState<Collector.StateType> {
    public HingeUpState() {
        super(Collector.StateType.HINGE_UP);
    }

    @Override
    public void execute() {
        robot.getCollector().setHingeServoPosition(Collector.HINGE_UP_POSITION);
        robot.getCollector().setSpindleMotorPower(0);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Collector.StateType.COLLECTING ||
                stateManager.getActiveStateType() == Collector.StateType.SPITTING;      // if want to hinge up collector while it is not done spitting, while assume that controller overrode spitting bc block already gone
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return time > Collector.HINGE_UP_TIME;
    }

    @Override
    public Collector.StateType getNextStateType() {
        return Collector.StateType.DONE_HINGING_UP;
    }
}
