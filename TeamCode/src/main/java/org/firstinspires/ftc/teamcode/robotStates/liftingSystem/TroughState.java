package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;


public class TroughState extends RobotState<LiftingSystem.StateType> {
    private boolean usingAutomaticTransfer;
    public TroughState() {
        super(LiftingSystem.StateType.TROUGH);
        usingAutomaticTransfer = true;
    }
    @Override
    public void execute(double dt) {
        // activating manual transfer
        if ((robot.getInput().getGamepadTracker2().isFirstFrameA() || robot.getInput().getGamepadTracker1().isFirstFrameA())
        && robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.IN && !robot.getGrabber().hasBlock())
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TRANSFER);
        // activating automatic transfer
        else if (usingAutomaticTransfer && robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.IN && robot.getCollector().getBlockColorSensor().hasValidatedColor())
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TRANSFER);

        // transitioning to drop area if need to
        if(robot.getInput().getGamepadTracker1().isFirstFrameB())
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_DROP_AREA);

        // handling safety overrides
        if ((robot.getInput().getGamepadTracker1().isFirstFrameX() || robot.getInput().getGamepadTracker2().isFirstFrameX())
        && robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.IN) {
            // resets arm position for transfer
            if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_SAFETY)
                robot.getArm().getTransitionState().setGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER);
            // knocks block
            else
                robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.KNOCK_BLOCK);
        }

        // handling what to do once have successful transfer
        if (robot.getGrabber().hasBlock()) {
            if (robot.getLiftingSystem().isDepositing()) {
                // prepping for basket deposit (automatically happens once your close enough to basket)
                if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER)
                    robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY, Arm.TRANSFER_TO_BASKET_SAFETY_TIME);
                // initiating basket transition on user input
                if (robot.getInput().getGamepadTracker2().isFirstFrameA())
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_BASKET);
            }

            // prepping for specimen pickup at human player station
            else
                robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_DROP_AREA);
        }
    }
    @Override
    public void executeOnExited() {
        usingAutomaticTransfer = true;
    }
    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.DROP_AREA_TO_TROUGH
                || robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.KNOCK_BLOCK
                || robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.TRANSFER;
    }

    // only can be overridden if lift is at trough safety and if grabber has block
    @Override
    public boolean canBeOverridden() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY;
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return null;
    }

    public void setUsingAutomaticTransfer(boolean usingAutomaticTransfer) {
        this.usingAutomaticTransfer = usingAutomaticTransfer;
    }
    @NonNull
    public String toString() {
        return toStringBase() +
                " | using auto transfer: " + usingAutomaticTransfer;
    }
}
