package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class MotorTransitionState<StateType extends Enum<StateType>> extends TransitionState<StateType> {
    private final DcMotorEx motor;
    private PIDController pid;
    private boolean usingPid;
    public final int DESTINATION_THRESHOLD;

    private int startEncoder;

    private int absoluteMin;
    private int absoluteMax;

    // stateType is enum of transition state type
    public MotorTransitionState(StateType stateType, DcMotorEx motor, int DESTINATION_THRESHOLD) {
        super(stateType);
        this.motor = motor;
        this.DESTINATION_THRESHOLD = DESTINATION_THRESHOLD;
        startEncoder = motor.getCurrentPosition();
        usingPid = false;
    }
    public MotorTransitionState(StateType stateType, DcMotorEx motor, int DESTINATION_THRESHOLD, PIDController pid) {
        super(stateType);
        this.motor = motor;
        this.DESTINATION_THRESHOLD = DESTINATION_THRESHOLD;
        this.pid = pid;
        usingPid = true;
        startEncoder = motor.getCurrentPosition();
    }
    public void setEncoderBounds(int min, int max) {
        absoluteMin = min;
        absoluteMax = max;
    }

    @Override
    public void setGoalState(double goalPosition, StateType goalStateType) {
        // only sets goal state the current state in stateManager is not in transition and if not already headed to goal state
        if(stateManager.getActiveStateType() != stateType & goalStateType != this.goalStateType) {
            if(pid != null) {
                usingPid = true;
                pid.reset();
                pid.setTarget(goalPosition);
            }
            startEncoder = motor.getCurrentPosition();
            this.goalPosition = goalPosition;
            this.goalStateType = goalStateType;
            stateManager.tryEnterState(stateType);
        }
    }
    public void setGoalStateWithoutPid(double goalPosition, StateType goalStateType) {
        // only sets goal state the current state in stateManager is not in transition and if not already headed to goal state
        if(stateManager.getActiveStateType() != stateType & goalStateType != this.goalStateType) {
            usingPid = false;
            startEncoder = motor.getCurrentPosition();
            this.goalPosition = goalPosition;
            this.goalStateType = goalStateType;
            stateManager.tryEnterState(stateType);
        }
    }
    @Override
    public void overrideGoalState(double goalPosition, StateType goalStateType) {
        if(pid != null) {
            usingPid = true;
            pid.reset();
            pid.setTarget(goalPosition);
        }
        startEncoder = motor.getCurrentPosition();
        this.goalPosition = goalPosition;
        this.goalStateType = goalStateType;
        stateManager.tryEnterState(stateType);
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
            if(pid == null || !usingPid)
                Subsystem.setMotorPosition(motor, (int) goalPosition);
            else
                Subsystem.setMotorPower(motor, pid.update(motor.getCurrentPosition()));
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
        int motorPos = motor.getCurrentPosition();
        int desiredDir = (int)goalPosition - startEncoder;
        int curDif = (int)goalPosition - motorPos;
        return Subsystem.inRange(motor, (int)goalPosition, DESTINATION_THRESHOLD);
                //|| Math.signum(desiredDir) != Math.signum(curDif); // checking if moving up, if difference is positive, vice versa for moving down
    }

    public PIDController getPid() {
        return pid;
    }
}
