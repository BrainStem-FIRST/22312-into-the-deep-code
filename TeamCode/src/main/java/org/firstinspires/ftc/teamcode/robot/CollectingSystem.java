package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.ExtendingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.InState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.OutState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.RetractingState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class CollectingSystem {

    public enum StateType {
        IN, EXTENDING, RETRACTING, OUT
    }

    private final BrainSTEMRobot robot;
    private final Gamepad gamepad1, gamepad2;
    private final StateManager<StateType> stateManager;
    public CollectingSystem(BrainSTEMRobot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        stateManager = new StateManager<>(StateType.IN);
        stateManager.addState(StateType.IN, new InState());
        stateManager.addState(StateType.OUT, new OutState());
        stateManager.addState(StateType.EXTENDING, new ExtendingState());
        stateManager.addState(StateType.RETRACTING, new RetractingState());
        stateManager.setupStates(robot, gamepad1, gamepad2, stateManager);
        stateManager.tryEnterState(StateType.IN);
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }

    public void update(double dt) {
        stateManager.update(dt);
    }
    public BrainSTEMRobot getRobot() {
        return robot;
    }
}