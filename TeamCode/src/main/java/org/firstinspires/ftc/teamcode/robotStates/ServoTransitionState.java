package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.Lift;

public class ServoTransitionState<StateType extends Enum<StateType>> extends TransitionState<StateType> {
    private final ServoImplEx servo;
    public ServoTransitionState(StateType stateType, ServoImplEx servo, double DESTINATION_THRESHOLD) {
        super(stateType, DESTINATION_THRESHOLD);
        this.servo = servo;
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
}
