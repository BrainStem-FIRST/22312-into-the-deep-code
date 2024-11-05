package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class MotorTransitionState<StateType extends Enum<StateType>> extends RobotState<StateType> {
    private int goalPosition;
    private StateType goalStateType;
    private final DcMotorEx motor;
    public final int DESTINATION_THRESHOLD;
    public MotorTransitionState(StateType stateType, DcMotorEx motor, int DESTINATION_THRESHOLD) {
        super(stateType);
        this.motor = motor;
        this.DESTINATION_THRESHOLD = DESTINATION_THRESHOLD;
    }
    @Override
    public void execute() {
        Subsystem.setMotorPosition(motor, goalPosition);
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
    public void setGoalState(int goalPosition, StateType goalStateType) {
        // only sets goal state the current state in stateManager is not in transition (because don't want to override that transition)
        if(stateManager.getActiveStateType() != stateType) {
            this.goalPosition = goalPosition;
            this.goalStateType = goalStateType;
            stateManager.tryEnterState(stateType);
        }
    }
}
