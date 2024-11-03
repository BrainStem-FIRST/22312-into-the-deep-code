package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.stateMachine.BaseState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.tele.BrainSTEMRobotTele;

public abstract class RobotState<StateType extends Enum<StateType>> extends BaseState<StateType> {

    protected BrainSTEMRobotTele robot;
    protected Gamepad gamepad;

    public RobotState(StateType stateType) {
        super(stateType);
    }

    @Override
    public void setup(Object... args) {
        robot = (BrainSTEMRobotTele) args[0];
        gamepad = (Gamepad) args[1];
        stateManager = (StateManager<StateType>) args[2];
    }
}
