package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.SearchAndCollectState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.SearchingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.InState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectingSystemStates.RetractingState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class CollectingSystem {

    public enum StateType {
        IN, SEARCH, SEARCH_AND_COLLECT, RETRACTING
    }

    private final BrainSTEMRobot robot;
    private final StateManager<StateType> stateManager;
    public CollectingSystem(BrainSTEMRobot robot) {
        this.robot = robot;

        stateManager = new StateManager<>(StateType.IN);
        stateManager.addState(StateType.IN, new InState());
        stateManager.addState(StateType.SEARCH, new SearchingState());
        stateManager.addState(StateType.SEARCH_AND_COLLECT, new SearchAndCollectState());
        stateManager.addState(StateType.RETRACTING, new RetractingState());

        stateManager.setupStates(robot, stateManager);
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

    public Action extendAndCollectAction(int extendMotorTick) {
        return new SequentialAction(
                getRobot().getExtension().extendAction(extendMotorTick),
                getRobot().getCollector().hingeDownAction(),
                getRobot().getCollector().collectAction(),
                getRobot().getCollector().hingeUpAction(),
                getRobot().getExtension().retractAction()
        );
    }
}