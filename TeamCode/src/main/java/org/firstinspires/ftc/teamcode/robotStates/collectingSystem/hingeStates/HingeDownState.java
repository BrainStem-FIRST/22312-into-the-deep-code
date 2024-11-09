package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.hingeStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class HingeDownState extends RobotState<Hinge.StateType> {

    public HingeDownState() {
        super(Hinge.StateType.HINGING_UP);
    }

    @Override
    public void execute() {
        robot.getHinge().setHingeServoPosition(Hinge.HINGE_DOWN_POSITION);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Hinge.StateType.UP;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return time > Hinge.HINGE_DOWN_TIME;
    }

    @Override
    public Hinge.StateType getNextStateType() {
        return Hinge.StateType.DOWN;
    }
}
