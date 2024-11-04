package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.stateMachine.BaseState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

public abstract class RobotState<StateType extends Enum<StateType>> extends BaseState<StateType> {

    protected BrainSTEMRobot robot;
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;

    public RobotState(StateType stateType) {
        super(stateType);
    }

    @Override
    public void setup(Object... args) {
        robot = (BrainSTEMRobot) args[0];
        gamepad1 = (Gamepad) args[1];
        gamepad2 = (Gamepad) args[2];
        stateManager = (StateManager<StateType>) args[3];
    }
}
