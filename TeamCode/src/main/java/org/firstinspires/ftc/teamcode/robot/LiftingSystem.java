package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates.*;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class LiftingSystem {
    private final BrainSTEMRobot robot;
    public enum StateType {
        TROUGH,
        TROUGH_TO_BASKET, BASKET_TO_BASKET, BASKET_DEPOSIT, BASKET_TO_TROUGH, // depositing block in basket
        TROUGH_TO_DROP_AREA, DROP_AREA, DROP_AREA_TO_RAM, RAM_TO_RAM, SPECIMEN_RAM, RAM_TO_TROUGH // ramming specimen on bar
    }
    private final StateManager<StateType> stateManager;

    public LiftingSystem(BrainSTEMRobot robot) {
        this.robot = robot;

        stateManager = new StateManager<>(StateType.TROUGH);

        stateManager.addState(StateType.TROUGH, new TroughState());
        stateManager.addState(StateType.TROUGH_TO_BASKET, new TroughToBasketState());
        stateManager.addState(StateType.BASKET_TO_BASKET, new BasketToBasketState());
        stateManager.addState(StateType.BASKET_DEPOSIT, new NothingState<>(StateType.BASKET_DEPOSIT));
        stateManager.addState(StateType.BASKET_TO_TROUGH, new BasketToTroughState());
        stateManager.addState(StateType.TROUGH_TO_DROP_AREA, new TroughToDropAreaState());
        stateManager.addState(StateType.DROP_AREA, new NothingState<>(StateType.DROP_AREA));
        stateManager.addState(StateType.DROP_AREA_TO_RAM, new DropAreaToRamState());
        stateManager.addState(StateType.SPECIMEN_RAM, new NothingState<>(StateType.SPECIMEN_RAM));
        stateManager.addState(StateType.RAM_TO_TROUGH, new RamToTroughState());

        stateManager.setupStates(robot, stateManager);
    }

    public void update(double dt) {
        stateManager.update(dt);

        // transitioning states if one state is done (bc I have non-transition states as NothingStates)

        // resetting lifting system once grabber releases block
        if(stateManager.getActiveStateType() == StateType.BASKET_DEPOSIT &&
                robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN)
            stateManager.tryEnterState(StateType.BASKET_TO_TROUGH);

        // once lift is done ramming
        else if(stateManager.getActiveStateType() == StateType.SPECIMEN_RAM &&
                robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_AFTER)
            // opening grabber once ram is done
            if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.OPENING);
                robot.getGrabber().setHasSpecimen(false);
                robot.setBlockColorHeld(BlockColor.NONE);
            }
            // resetting lifting system once block is released
            else if(!robot.getGrabber().getHasSpecimen())
                stateManager.tryEnterState(StateType.RAM_TO_TROUGH);

    }
    public BrainSTEMRobot getRobot() {
        return robot;
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }
}
