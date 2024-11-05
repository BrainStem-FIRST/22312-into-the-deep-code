package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.stateMachine.BaseState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public abstract class RobotState<StateType extends Enum<StateType>> extends BaseState<StateType> {

    protected BrainSTEMRobot robot;
    protected Input input;

    public RobotState(StateType stateType) {
        super(stateType);
    }

    @Override
    public void setup(Object... args) {
        robot = (BrainSTEMRobot) args[0];
        input = (Input) args[1];
        stateManager = (StateManager<StateType>) args[2];
    }
}
