package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates.CollectState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates.ExtendingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates.InState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates.RetractingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates.SpitState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class CollectingSystemTele {

    public enum StateType {
        IN, EXTENDING, RETRACTING, COLLECTING, SPITTING
    }
    private final StateManager<StateType> stateManager;
    public CollectingSystemTele(BrainSTEMRobotTele robot, Gamepad gamepad) {
        stateManager = new StateManager<>(StateType.IN);

        stateManager.addState(StateType.IN, new InState());
        stateManager.addState(StateType.EXTENDING, new ExtendingState());
        stateManager.addState(StateType.RETRACTING, new RetractingState());
        stateManager.addState(StateType.COLLECTING, new CollectState());
        stateManager.addState(StateType.SPITTING, new SpitState());

        stateManager.setupStates(robot, gamepad);
        stateManager.tryEnterState(StateType.IN);
    }

    public StateManager<StateType> getStateManager() { return stateManager; }

    public void update(double dt) {
        stateManager.update(dt);
    }
}
