package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates.*;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class LiftingSystem {
    private final BrainSTEMRobot robot;
    public enum StateType {
        TROUGH,
        TROUGH_TO_BASKET, BASKET_DEPOSIT, BASKET_TO_TROUGH, // depositing block in basket
        TROUGH_TO_DROP_AREA, DROP_AREA, DROP_AREA_TO_RAM, SPECIMEN_RAM, RAM_TO_TROUGH // ramming specimen on bar
    }
    private final StateManager<StateType> stateManager;
    public LiftingSystem(BrainSTEMRobot robot) {
        this.robot = robot;

        stateManager = new StateManager<>(StateType.TROUGH);

        stateManager.addState(StateType.TROUGH, new TroughState());
        stateManager.addState(StateType.TROUGH_TO_BASKET, new TroughToBasketState(Lift.DESTINATION_THRESHOLD));
        stateManager.addState(StateType.BASKET_DEPOSIT, new NothingState<>(StateType.BASKET_DEPOSIT));
        stateManager.addState(StateType.BASKET_TO_TROUGH, new BasketToTroughState());
        stateManager.addState(StateType.TROUGH_TO_DROP_AREA, new TroughToDropAreaState());
        stateManager.addState(StateType.DROP_AREA, new NothingState<>(StateType.DROP_AREA));
        stateManager.addState(StateType.DROP_AREA_TO_RAM, new DropAreaToRamState());
        stateManager.addState(StateType.SPECIMEN_RAM, new NothingState<>(StateType.SPECIMEN_RAM));
        stateManager.addState(StateType.RAM_TO_TROUGH, new RamToTroughState<>());

        stateManager.setupStates(robot, stateManager);
        stateManager.tryEnterState(StateType.TROUGH);
    }
    public void update(double dt) {
        stateManager.update(dt);
    }
    public BrainSTEMRobot getRobot() {
        return robot;
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }
}
