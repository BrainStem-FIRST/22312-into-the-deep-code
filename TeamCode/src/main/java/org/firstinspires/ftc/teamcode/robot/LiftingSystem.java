package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.*;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.Helper;

public class LiftingSystem {
    private final BrainSTEMRobot robot;
    public enum StateType {
        TROUGH, TROUGH_TO_DROP_AREA, DROP_AREA, DROP_AREA_TO_TROUGH, // "default states"
        TROUGH_TO_BASKET, BASKET_TO_BASKET, BASKET_DEPOSIT, BASKET_TO_DROP_AREA, // depositing block in basket states
        DROP_AREA_TO_RAM, SPECIMEN_RAM, RAM_TO_DROP_AREA // ramming specimen on bar states
    }

    // for depositing can have radial safety check from basket (bc only approach from one corner)
    public static final Vector2d DEPOSIT_SAFETY_POS = new Vector2d(-53, -53);
    public static final Vector2d DEPOSIT_CORNER = new Vector2d(-72, -72);
    public static final double DEPOSIT_SAFETY_DIST = Helper.dist(DEPOSIT_SAFETY_POS, DEPOSIT_CORNER);
    private boolean resetToTrough;
    private boolean buttonACued; // if a is cued during transition, an action should automatically occur once transition is done
    private final StateManager<StateType> stateManager;

    public LiftingSystem(BrainSTEMRobot robot) {
        this.robot = robot;

        stateManager = new StateManager<>(StateType.DROP_AREA);

        stateManager.addState(StateType.TROUGH, new TroughState());
        stateManager.addState(StateType.TROUGH_TO_BASKET, new TroughToBasketState());
        stateManager.addState(StateType.BASKET_TO_BASKET, new BasketToBasketState());
        stateManager.addState(StateType.BASKET_DEPOSIT, new NothingState<>(StateType.BASKET_DEPOSIT));
        stateManager.addState(StateType.BASKET_TO_DROP_AREA, new BasketToDropAreaState());
        stateManager.addState(StateType.TROUGH_TO_DROP_AREA, new TroughToDropAreaState());
        stateManager.addState(StateType.DROP_AREA, new DropAreaState());
        stateManager.addState(StateType.DROP_AREA_TO_TROUGH, new DropAreaToTroughState());
        stateManager.addState(StateType.DROP_AREA_TO_RAM, new DropAreaToRamState());
        stateManager.addState(StateType.SPECIMEN_RAM, new NothingState<>(StateType.SPECIMEN_RAM));
        stateManager.addState(StateType.RAM_TO_DROP_AREA, new RamToDropAreaState());

        stateManager.setupStates(robot, stateManager);

        resetToTrough = false;
        buttonACued = false;
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
    public boolean isResetToTrough() {
        return resetToTrough;
    }
    public void setResetToTrough(boolean resetToTrough) {
        this.resetToTrough = resetToTrough;
    }
    public boolean isButtonACued() {
        return buttonACued;
    }
    public void setButtonACued(boolean buttonACued) {
        this.buttonACued = buttonACued;
    }

    public boolean subsystemsAtDropArea() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA
                && robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DROP_OFF;
    }
    public boolean subsystemsAtTrough() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY
                && robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER;
    }

    public Action transferBlock() {
        return new SequentialAction(
            robot.getLift().moveTo(Lift.TROUGH_POS),
            robot.getGrabber().close(),
            robot.getLift().moveTo(Lift.TROUGH_SAFETY_POS)
        );
    }
    public Action transferToDropOff() {
        return new SequentialAction(
                robot.getArm().rotateTo(Arm.DROP_OFF_POS, Arm.TRANSFER_TO_DROP_OFF_TIME),
                robot.getLift().moveTo(Lift.DROP_AREA_POS)
        );
    }
    public Action depositHigh() {
        return new SequentialAction(
            robot.getArm().rotateTo(Arm.BASKET_SAFETY_POS, 0.4),
            robot.getLift().moveTo(Lift.HIGH_BASKET_SAFETY_POS),
            new ParallelAction(
                    robot.getArm().rotateTo(Arm.BASKET_DROP_POS, 0.4),
                    robot.getLift().moveTo(Lift.HIGH_BASKET_POS),
                    new SequentialAction(
                            new SleepAction(0.2),
                            robot.getGrabber().open()
                    )
            )
        );
    }
    public Action lowerFromDeposit() {
        return new SequentialAction(
                robot.getArm().rotateTo(Arm.BASKET_SAFETY_POS, 0.3),
                robot.getLift().moveTo(Lift.TROUGH_SAFETY_POS),
                robot.getArm().rotateTo(Arm.TRANSFER_POS, Arm.TRANSFER_TO_BASKET_SAFETY_TIME)
        );
    }

    // specimen actions
    public Action setupHighSpecimenRam() {
        return new SequentialAction(
                robot.getLift().moveTo(Lift.TROUGH_SAFETY_POS),
                new ParallelAction(
                    robot.getLift().moveTo(Lift.HIGH_RAM_BEFORE_POS),
                    robot.getArm().rotateTo(Arm.SPECIMEN_RAM_POS, Arm.UP_TO_TRANSFER_TIME + Arm.SPECIMEN_RAM_TO_UP_TIME)
                )
        );
    }
    public Action ramHighSpecimen() {
        return new SequentialAction(
                robot.getLift().moveTo(Lift.HIGH_RAM_AFTER_POS),
                robot.getGrabber().open()
        );
    }
    public Action resetSpecimenRam() {
        return new SequentialAction(
            robot.getArm().rotateTo(Arm.UP_POS, Arm.SPECIMEN_RAM_TO_UP_TIME),
            new ParallelAction(
                robot.getArm().rotateTo(Arm.TRANSFER_POS, Arm.UP_TO_TRANSFER_TIME),
                robot.getLift().moveTo(Lift.TROUGH_SAFETY_POS))
        );
    }
}
