package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class MotorTransitionState<StateType extends Enum<StateType>> extends TransitionState<StateType> {
    private final DcMotorEx motor;
    private double powerOffset;
    private PIDController pid;
    private boolean usingPid;
    public final int DESTINATION_THRESHOLD;

    private int startEncoder;
    private double maxTime;

    private int absoluteMin;
    private int absoluteMax;

    // stateType is enum of transition state type
    public MotorTransitionState(StateType stateType, DcMotorEx motor, int DESTINATION_THRESHOLD) {
        super(stateType);
        this.motor = motor;
        this.DESTINATION_THRESHOLD = DESTINATION_THRESHOLD;
        powerOffset = 0;
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
    public void setMaxTimeThreshold(double time) {
        maxTime = time;
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
                // finding pid power + offset, and ensuring it stays between -1 and 1
                double totalPower = Range.clip(pid.update(motor.getCurrentPosition()) + powerOffset, -1, 1);
                Subsystem.setMotorPower(motor, totalPower);
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
        return Subsystem.inRange(motor, (int)goalPosition, DESTINATION_THRESHOLD);
    }

    public PIDController getPid() {
        return pid;
    }
    public void setPowerOffset(double powerOffset) {
        this.powerOffset = powerOffset;
    }
    public int getDirection() {
        return (int) Math.signum(getGoalStatePosition() - motor.getCurrentPosition());
    }
}
