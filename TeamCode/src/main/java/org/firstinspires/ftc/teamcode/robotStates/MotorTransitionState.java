package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class MotorTransitionState<StateType extends Enum<StateType>> extends TransitionState<StateType> {
    private final DcMotorEx motor;
    public MotorTransitionState(StateType stateType, DcMotorEx motor, int DESTINATION_THRESHOLD) {
        super(stateType, DESTINATION_THRESHOLD);
        this.motor = motor;
    }
    @Override
    public void execute() {
        Subsystem.setMotorPosition(motor, (int) goalPosition);
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
        return Math.abs(motor.getCurrentPosition() - goalPosition) < DESTINATION_THRESHOLD;
    }

    @Override
    public StateType getNextStateType() {
        return goalStateType;
    }
}
