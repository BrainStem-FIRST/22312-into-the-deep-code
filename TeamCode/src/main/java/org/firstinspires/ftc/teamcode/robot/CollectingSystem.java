package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.SearchAndCollectState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.SearchingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.InState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.RetractingState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public class CollectingSystem {

    public enum StateType {
        IN, SEARCH, SEARCH_AND_COLLECT, RETRACTING
    }

    private final BrainSTEMRobot robot;
    private final StateManager<StateType> stateManager;
    public CollectingSystem(BrainSTEMRobot robot, Input input) {
        this.robot = robot;

        stateManager = new StateManager<>(StateType.IN);
        stateManager.addState(StateType.IN, new InState());
        stateManager.addState(StateType.SEARCH, new SearchingState());
        stateManager.addState(StateType.SEARCH_AND_COLLECT, new SearchAndCollectState());
        stateManager.addState(StateType.RETRACTING, new RetractingState());

        stateManager.setupStates(robot, input, stateManager);
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