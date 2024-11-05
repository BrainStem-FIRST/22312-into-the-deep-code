package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates.BasketDepositState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates.BasketToTroughState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates.DropAreaState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates.DropAreaToRamState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates.RamToTroughState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates.SpecimenRamState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates.TroughState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates.TroughToBasketState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates.TroughToDropAreaState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public class LiftingSystem {
    private final BrainSTEMRobot robot;
    private final Input input;
    public enum StateType {
        TROUGH, TROUGH_TO_BASKET, BASKET_DEPOSIT, BASKET_TO_TROUGH, TROUGH_TO_DROP_AREA, DROP_AREA, DROP_AREA_TO_RAM, SPECIMEN_RAM, RAM_TO_TROUGH
    }
    private final StateManager<StateType> stateManager;
    public LiftingSystem(BrainSTEMRobot robot, Input input) {
        this.robot = robot;
        this.input = input;

        stateManager = new StateManager<>(StateType.TROUGH);

        stateManager.addState(StateType.TROUGH, new TroughState());
        stateManager.addState(StateType.TROUGH_TO_BASKET, new TroughToBasketState());
        stateManager.addState(StateType.BASKET_DEPOSIT, new BasketDepositState());
        stateManager.addState(StateType.BASKET_TO_TROUGH, new BasketToTroughState());
        stateManager.addState(StateType.TROUGH_TO_DROP_AREA, new TroughToDropAreaState());
        stateManager.addState(StateType.DROP_AREA, new DropAreaState());
        stateManager.addState(StateType.DROP_AREA_TO_RAM, new DropAreaToRamState());
        stateManager.addState(StateType.SPECIMEN_RAM, new SpecimenRamState());
        stateManager.addState(StateType.RAM_TO_TROUGH, new RamToTroughState());

        stateManager.setupStates(robot, input, stateManager);
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
