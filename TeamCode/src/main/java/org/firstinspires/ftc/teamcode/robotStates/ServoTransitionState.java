package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.Lift;

public class ServoTransitionState<StateType extends Enum<StateType>> extends TransitionState<StateType> {
    private final ServoImplEx servo;
    private final double timeDone;
    public ServoTransitionState(StateType stateType, ServoImplEx servo) {
        super(stateType);
        this.servo = servo;
        timeDone = 0.5;
    }
    public ServoTransitionState(StateType stateType, ServoImplEx servo, double timeDone) {
        super(stateType);
        this.servo = servo;
        this.timeDone = timeDone;
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
        return time > timeDone;
    }

    @Override
    public StateType getNextStateType() {
        return goalStateType;
    }
}
