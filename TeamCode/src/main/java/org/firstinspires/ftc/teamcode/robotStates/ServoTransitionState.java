package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class ServoTransitionState<StateType extends Enum<StateType>> extends TransitionState<StateType> {
    private final ServoImplEx servo;
    private double timeDone;
    public final double FULL_ROTATION_TIME;
    public ServoTransitionState(StateType stateType, ServoImplEx servo) {
        super(stateType);
        this.servo = servo;
        this.FULL_ROTATION_TIME = 1;
    }
    public ServoTransitionState(StateType stateType, ServoImplEx servo, double fullRotationTime) {
        super(stateType);
        this.servo = servo;
        this.FULL_ROTATION_TIME = fullRotationTime;
    }

    @Override
    public void setGoalState(double goalPosition, StateType goalStateType) {
        // only sets goal state the current state in stateManager is not in transition and if not already headed to goal state
        if(stateManager.getActiveStateType() != stateType & goalStateType != this.goalStateType) {
            this.timeDone = Subsystem.getServoTime(this.goalPosition, goalPosition, FULL_ROTATION_TIME);
            this.goalPosition = goalPosition;
            this.goalStateType = goalStateType;
            stateManager.tryEnterState(stateType);
        }
    }

    // allows for manual setting of time to finish
    public void setGoalState(double goalPosition, StateType goalStateType, double timeDone) {
        // only sets goal state the current state in stateManager is not in transition and if not already headed to goal state
        if(stateManager.getActiveStateType() != stateType & goalStateType != this.goalStateType) {
            this.timeDone = timeDone;
            this.goalPosition = goalPosition;
            this.goalStateType = goalStateType;
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
