package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftStates;

import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class TransitionState extends RobotState<Lift.StateType> {
    private int goalPosition;
    public TransitionState() {
        super(Lift.StateType.TRANSITION);
    }
    @Override
    public void execute() {
        robot.getLift().setLiftPosition(goalPosition);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Lift.StateType.STATIC;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return Math.abs(robot.getLift().getLiftMotor().getCurrentPosition() - goalPosition) < Lift.DESTINATION_THRESHOLD;
    }

    @Override
    public Lift.StateType getNextStateType() {
        return Lift.StateType.STATIC;
    }
    public void setGoalPosition(int goalPosition) {
        this.goalPosition = goalPosition;
    }
}
