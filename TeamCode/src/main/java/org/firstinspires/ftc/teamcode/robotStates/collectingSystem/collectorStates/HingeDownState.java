package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class HingeDownState extends RobotState<Collector.StateType> {

    public HingeDownState() {
        super(Collector.StateType.HINGE_DOWN);
    }

    @Override
    public void execute() {
        robot.getCollector().setHingeServoPosition(Collector.HINGE_DOWN_POSITION);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Collector.StateType.READY_TO_HINGE_DOWN;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return time > Collector.HINGE_DOWN_TIME;
    }

    @Override
    public Collector.StateType getNextStateType() {
        return Collector.StateType.COLLECTING;
    }
}
