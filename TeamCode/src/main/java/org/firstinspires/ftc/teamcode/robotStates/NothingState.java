package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Subsystem;

import java.util.ArrayList;

public class NothingState<StateType extends Enum<StateType>> extends RobotState<StateType> {

    ArrayList<DcMotorEx> motorList;
    public NothingState(StateType stateType) {
        super(stateType);
        motorList = new ArrayList<>();
    }
    public NothingState(StateType stateType, DcMotorEx motor) {
        super(stateType);
        motorList = new ArrayList<>();
        motorList.add(motor);
    }

    public void addMotor(DcMotorEx motor) {
        motorList.add(motor);
    }

    @Override
    public void execute() {
        for (DcMotorEx motor : motorList) {
            Subsystem.setMotorPower(motor, 0);
        }
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() != stateType;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return false;
    }

    // should never be called bc nothing state will never be done; is meant to be overridden
    @Override
    public StateType getNextStateType() {
        return null;
    }
}
