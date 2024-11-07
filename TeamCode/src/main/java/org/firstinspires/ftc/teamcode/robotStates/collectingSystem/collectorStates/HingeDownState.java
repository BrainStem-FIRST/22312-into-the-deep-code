package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class HingeDownState extends RobotState<Collector.StateType> {

    public static double HINGE_DOWN_TIME = 1;
    public HingeDownState() {
        super(Collector.StateType.HINGE_DOWN);
    }

    @Override
    public void execute() {
        robot.telemetry.addData("inside execute function in hinge_down state; time running", time);
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
        return time > HINGE_DOWN_TIME;
        //return Math.abs(robot.getCollector().getHingeServo().getPosition() - Collector.HINGE_DOWN_POSITION) < Collector.HINGE_THRESHOLD;
    }

    @Override
    public Collector.StateType getNextStateType() {
        return Collector.StateType.COLLECTING;
    }
}
