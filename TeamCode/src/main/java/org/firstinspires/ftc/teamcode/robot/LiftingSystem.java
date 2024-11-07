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
    public enum StateType {
        TROUGH, TO_TROUGH_TRANSITION, 
        BASKET_DEPOSIT, TO_BASKET_TRANSITION, // depositing block in basket
        TROUGH_TO_DROP_AREA, DROP_AREA, TO_RAM_TRANSITION, SPECIMEN_RAM // ramming specimen on bar
    }
    private final StateManager<StateType> stateManager;
    public LiftingSystem(BrainSTEMRobot robot) {
        this.robot = robot;

        stateManager = new StateManager<>(StateType.TROUGH);

        stateManager.addState(StateType.TROUGH, new TroughState());
        stateManager.addState(StateType.TO_TROUGH_TRANSITION, new ToTroughTransitionState());

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
