package org.firstinspires.ftc.teamcode.robotStates;

import org.firstinspires.ftc.teamcode.stateMachine.BaseState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

public abstract class RobotState<StateType extends Enum<StateType>> extends BaseState<StateType> {

    protected BrainSTEMRobot robot;
    protected StateManager<StateType> stateManager;

    public RobotState(StateType stateType) {
        super(stateType);
    }

    @Override
    public void setup(Object... args) {
        // Ensure exactly two parameters are passed
        if (args.length != 2) {
            throw new IllegalArgumentException("RobotState.setup() requires exactly two arguments.");
        }
        robot = (BrainSTEMRobot) args[0];
        stateManager = (StateManager<StateType>) args[1];
    }
}
