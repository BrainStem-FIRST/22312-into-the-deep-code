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

        // run spindles at slow speed to hold block in place while hinge happens
        robot.getCollector().setSpindleMotorPower(Collector.HOLD_SPIN_POWER);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Collector.StateType.COLLECTING ||
                stateManager.getActiveStateType() == Collector.StateType.SPITTING;      // if want to hinge up collector while it is not done spitting, while assume that controller overrode spitting bc block already gone
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return Math.abs(robot.getCollector().getHingeServo().getPosition() - Collector.HINGE_UP_POSITION) < Collector.HINGE_THRESHOLD;
    }

    @Override
    public Collector.StateType getNextStateType() {
        return Collector.StateType.NOTHING;
    }
}
