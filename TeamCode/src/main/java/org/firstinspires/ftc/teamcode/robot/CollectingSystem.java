package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates.ExtendingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates.InState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates.RetractingState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.tele.BrainSTEMRobotTele;

public class CollectingSystem {

    public enum StateType {
        IN, EXTENDING, RETRACTING, OUT
    }

    private final BrainSTEMRobotTele robot;
    private final Gamepad gamepad;
    private final StateManager<StateType> stateManager;
    public CollectingSystem(BrainSTEMRobotTele robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;

        stateManager = new StateManager<>(StateType.IN);
        stateManager.addState(StateType.IN, new InState());
        stateManager.addState(StateType.OUT, new InState());
        stateManager.addState(StateType.EXTENDING, new ExtendingState());
        stateManager.addState(StateType.RETRACTING, new RetractingState());
        stateManager.setupStates(robot, gamepad);
        stateManager.tryEnterState(StateType.IN);
    }

    public StateManager<StateType> getStateManager() { return stateManager; }

    public void update(double dt) {

        // a extends and retracts collector
        if (gamepad.a)
            if (!stateManager.tryEnterState(StateType.EXTENDING))
                stateManager.tryEnterState(StateType.RETRACTING);

        stateManager.update(dt);
    }
}
