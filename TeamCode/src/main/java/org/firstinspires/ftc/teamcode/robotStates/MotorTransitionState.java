package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class MotorTransitionState<StateType extends Enum<StateType>> extends TransitionState<StateType> {
    private final DcMotorEx motor;
    public final int DESTINATION_THRESHOLD;

    private int absoluteMin;
    private int absoluteMax;

    // stateType is enum of transition state type
    public MotorTransitionState(StateType stateType, DcMotorEx motor, int DESTINATION_THRESHOLD) {
        super(stateType);
        this.motor = motor;
        this.DESTINATION_THRESHOLD = DESTINATION_THRESHOLD;
    }
    public void setEncoderBounds(int min, int max) {
        absoluteMin = min;
        absoluteMax = max;
    }

    @Override
    public void execute() {
        // checking hardstops
        if(motor.getCurrentPosition() < absoluteMin)
            Subsystem.setMotorPosition(motor, absoluteMin);
        else if(motor.getCurrentPosition() > absoluteMax)
            Subsystem.setMotorPosition(motor, absoluteMax);

        // running regular motor
        else
            Subsystem.setMotorPosition(motor, (int) goalPosition);
            //Subsystem.setMotorPower(motor, Math.signum(goalPosition - motor.getCurrentPosition()) * power);

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
