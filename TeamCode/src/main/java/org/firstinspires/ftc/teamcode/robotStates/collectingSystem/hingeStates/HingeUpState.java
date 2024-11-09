package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.hingeStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;
public class HingeUpState extends RobotState<Hinge.StateType> {
    public HingeUpState() {
        super(Hinge.StateType.HINGING_UP);
    }

    @Override
    public void execute() {
        robot.getHinge().setHingeServoPosition(Hinge.HINGE_UP_POSITION);
        robot.getCollector().setSpindleMotorPower(0);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Hinge.StateType.DOWN;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return time > Hinge.HINGE_UP_TIME;
    }

    @Override
    public Hinge.StateType getNextStateType() {
        return Hinge.StateType.UP;
    }
}
