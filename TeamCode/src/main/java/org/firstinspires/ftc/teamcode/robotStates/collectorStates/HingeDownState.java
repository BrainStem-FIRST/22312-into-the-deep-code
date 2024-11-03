package org.firstinspires.ftc.teamcode.robotStates.collectorStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class HingeDownState extends RobotState<Collector.StateType> {
    public HingeDownState() {
        super(Collector.StateType.HINGE_UP);
    }

    @Override
    public void execute() {
        robot.getCollector().setHingeServoPosition(Collector.HINGE_DOWN_POSITION);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Collector.StateType.NOTHING ||
                stateManager.getActiveStateType() == Collector.StateType.HINGE_UP;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return Math.abs(robot.getCollector().getHingeServo().getPosition() - Collector.HINGE_DOWN_POSITION) < Collector.HINGE_THRESHOLD;
    }

    @Override
    public Collector.StateType getNextStateType() {
        return Collector.StateType.COLLECTING;
    }
}
