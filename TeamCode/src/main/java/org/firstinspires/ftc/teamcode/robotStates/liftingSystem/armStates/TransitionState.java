package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.armStates;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class TransitionState extends RobotState<Arm.StateType> {
    private double goalPosition;
    public TransitionState() {
        super(Arm.StateType.TRANSITION);
    }
    @Override
    public void execute() {
        robot.getArm().getArmServo().setPosition(goalPosition);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() != Arm.StateType.TRANSITION;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return Math.abs(robot.getArm().getArmServo().getPosition() - goalPosition) < Arm.DESTINATION_THRESHOLD;
    }

    @Override
    public Arm.StateType getNextStateType() {
        return Arm.getState(goalPosition);
    }
    public void setGoalPosition(double goalPosition) {
        this.goalPosition = goalPosition;
    }
}
