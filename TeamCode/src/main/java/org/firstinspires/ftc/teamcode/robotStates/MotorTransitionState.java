package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class MotorTransitionState<StateType extends Enum<StateType>> extends TransitionState<StateType> {
    private final DcMotorEx motor;
    private PIDController pid;
    private boolean usingPid;
    public final int DESTINATION_THRESHOLD;
    private double maxTime;

    private int absoluteMin;
    private int absoluteMax;
    // if true, returns done during transition as soon as encoder passes goal position; doesn't take into account whether subsystem is in range or not
    // Note: reset to false at the end of every transition
    private boolean tempDoneWhenPassPosition;
    private int startPos;

    // stateType is enum of transition state type
    public MotorTransitionState(StateType stateType, DcMotorEx motor, int DESTINATION_THRESHOLD) {
        super(stateType);
        this.motor = motor;
        this.DESTINATION_THRESHOLD = DESTINATION_THRESHOLD;
        usingPid = false;
        tempDoneWhenPassPosition = false;
        startPos = motor.getCurrentPosition();
    }
    public MotorTransitionState(StateType stateType, DcMotorEx motor, int DESTINATION_THRESHOLD, PIDController pid) {
        super(stateType);
        this.motor = motor;
        this.DESTINATION_THRESHOLD = DESTINATION_THRESHOLD;
        this.pid = pid;
        usingPid = true;
        tempDoneWhenPassPosition = false;
        startPos = motor.getCurrentPosition();
    }
    public void setMaxTimeThreshold(double time) {
        maxTime = time;
    }
    public void setEncoderBounds(int min, int max) {
        absoluteMin = min;
        absoluteMax = max;
    }
    @Override
    public void executeOnExited() {
        if(pid != null)
            pid.reset();
        tempDoneWhenPassPosition = false;
    }
    @Override
    public void setGoalState(double goalPosition, StateType goalStateType) {
        // only sets goal state the current state in stateManager is not in transition and if not already headed to goal state
        if(stateManager.getActiveStateType() != stateType & goalStateType != this.goalStateType) {
            if(pid != null) {
                usingPid = true;
                pid.setTarget(goalPosition);
            }
            this.startPos = motor.getCurrentPosition();
            this.goalPosition = goalPosition;
            this.goalStateType = goalStateType;
            stateManager.tryEnterState(stateType);
        }
    }
    public void setGoalStateWithoutPid(double goalPosition, StateType goalStateType) {
        // only sets goal state the current state in stateManager is not in transition and if not already headed to goal state
        if(stateManager.getActiveStateType() != stateType & goalStateType != this.goalStateType) {
            usingPid = false;
            this.startPos = motor.getCurrentPosition();
            this.goalPosition = goalPosition;
            this.goalStateType = goalStateType;
            stateManager.tryEnterState(stateType);
        }
    }
    @Override
    public void overrideGoalState(double goalPosition, StateType goalStateType) {
        if(usingPid)
            pid.setTarget(goalPosition);

        this.startPos = motor.getCurrentPosition();
        this.goalPosition = goalPosition;
        this.goalStateType = goalStateType;
        stateManager.tryEnterState(stateType);
    }

    @Override
    public void execute(double dt) {

        if(maxTime != 0 && getTime() >= maxTime)
            stateManager.tryEnterState(goalStateType);
        // checking hardstops
        if(motor.getCurrentPosition() < absoluteMin)
            Subsystem.setMotorPosition(motor, absoluteMin);
        else if(motor.getCurrentPosition() > absoluteMax)
            Subsystem.setMotorPosition(motor, absoluteMax);

        // running regular motor
        else
            if(pid == null || !usingPid)
                Subsystem.setMotorPosition(motor, (int) goalPosition);
            else {
                Subsystem.setMotorPower(motor, pid.update(motor.getCurrentPosition()));
            }
    }
    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() != stateType; // stateType should equal TRANSITION enum
    }
    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        if(tempDoneWhenPassPosition)
            return Math.signum(motor.getCurrentPosition() - goalPosition) == Math.signum(goalPosition - startPos);
        return Subsystem.inRange(motor, (int)goalPosition, DESTINATION_THRESHOLD);
    }

    public PIDController getPid() {
        return pid;
    }
    public int getDirection() {
        return (int) Math.signum(getGoalStatePosition() - motor.getCurrentPosition());
    }
    public boolean isUsingPid() {
        return usingPid;
    }
    public void setDoneWhenPassPosition() {
        tempDoneWhenPassPosition = true;
    }
}
