package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.Lift;

public class ServoTransitionState<StateType extends Enum<StateType>> extends TransitionState<StateType> {
    private final ServoImplEx servo;
    private double timeDone;
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

    // NOTE: am NOT overriding the setGoalState in TransitionState, just providing alternative if one wants to change the timeDone
    public void setGoalState(double goalPosition, StateType goalStateType, double timeDone) {
        // only sets goal state the current state in stateManager is not in transition and if not already headed to goal state
        if(stateManager.getActiveStateType() != stateType & goalStateType != this.goalStateType) {
            this.goalPosition = goalPosition;
            this.goalStateType = goalStateType;
            this.timeDone = timeDone;
            stateManager.tryEnterState(stateType);
        }
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
