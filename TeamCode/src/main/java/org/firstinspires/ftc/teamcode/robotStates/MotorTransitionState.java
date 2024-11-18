package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class MotorTransitionState<StateType extends Enum<StateType>> extends TransitionState<StateType> {
    private final DcMotorEx motor;
    private final PIDController pid;

    private int absoluteMin;
    private int absoluteMax;

    // stateType is enum of transition state type
    public MotorTransitionState(StateType stateType, DcMotorEx motor, int DESTINATION_THRESHOLD) {
        super(stateType, DESTINATION_THRESHOLD);
        this.motor = motor;
        this.pid = null;
    }
    public MotorTransitionState(StateType stateType, DcMotorEx motor, int DESTINATION_THRESHOLD, PIDController pid) {
        super(stateType, DESTINATION_THRESHOLD);
        this.motor = motor;
        this.pid = pid;
    }
    public void setEncoderBounds(int min, int max) {
        absoluteMin = min;
        absoluteMax = max;
    }
    @Override
    public void execute() {
        if(motor.getCurrentPosition() > absoluteMin && motor.getCurrentPosition() < absoluteMax)
            if(pid != null) {
                if (isFirstTime()) {
                    pid.setTarget(goalPosition);
                    pid.reset();
                }
                Subsystem.setMotorPower(motor, pid.update(motor.getCurrentPosition()));
            }
            else
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
