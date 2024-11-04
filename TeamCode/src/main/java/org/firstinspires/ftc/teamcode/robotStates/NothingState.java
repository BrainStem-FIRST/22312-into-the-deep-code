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

    @Override
    public StateType getNextStateType() {
        return null;
    }
}
