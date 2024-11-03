package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.Lift;

public class ServoTransitionState<StateType extends Enum<StateType>> extends RobotState<StateType> {
    private double goalPosition;
    private StateType goalStateType;
    private final ServoImplEx servo;
    public final double DESTINATION_THRESHOLD;
    public ServoTransitionState(StateType transition, ServoImplEx servo, double DESTINATION_THRESHOLD) {
        super(transition);
        this.servo = servo;
        this.DESTINATION_THRESHOLD = DESTINATION_THRESHOLD;
    }
    @Override
    public void execute() {
        servo.setPosition(goalPosition);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() != stateType; // stateType should equal TRANSITION enum
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return Math.abs(servo.getPosition() - goalPosition) < DESTINATION_THRESHOLD;
    }

    @Override
    public StateType getNextStateType() {
        return goalStateType;
    }
    public void setGoalState(double goalPosition, StateType goalStateType) {
        this.goalPosition = goalPosition;
        this.goalStateType = goalStateType;
    }
}
