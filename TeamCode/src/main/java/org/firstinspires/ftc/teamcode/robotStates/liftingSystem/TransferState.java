package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class TransferState extends RobotState<LiftingSystem.StateType> {
    private boolean transferredOnce;
    public TransferState() {
        super(LiftingSystem.StateType.TRANSFER);
    }
    @Override
    public void executeOnEntered() {
        transferredOnce = false;
        // setting up arm just in case
        robot.getArm().getTransitionState().setGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER);
    }
    @Override
    public void execute(double dt) {
        // checking for override conditions (overrides transfer when tries to extend, spit/collect, or when x is pressed)
        if(robot.getInput().getGamepadTracker1().isFirstFrameRightTrigger()
        || robot.getInput().getGamepadTracker1().isFirstFrameX() || robot.getInput().getGamepadTracker2().isFirstFrameX()
        || robot.getInput().getGamepadTracker1().isFirstFrameDpadUp() || robot.getInput().getGamepadTracker1().isFirstFrameDpadDown()) {
            robot.getLift().getTransitionState().overrideGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
            robot.getArm().getTransitionState().overrideGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER);
            robot.getGrabber().getTransitionState().overrideGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
            transferredOnce = true;
            // telling trough state to break automatic transfer loop
            ((TroughState) robot.getLiftingSystem().getStateManager().getState(LiftingSystem.StateType.TROUGH)).setUsingAutomaticTransfer(false);
        }

        // executing actual transfer
        else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER) {
            // moving lift down
            if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {
                robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_POS, Lift.StateType.TROUGH);
                robot.getLift().getTransitionState().setMaxTimeThreshold(Lift.MAX_TRANSFER_TIME);
                robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
            } else if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH)
                // grabbing block
                if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                    robot.getGrabber().getTransitionState().setGoalState(Grabber.CLOSE_POS, Grabber.StateType.CLOSED);
                    robot.getGrabber().setBlockColorHeld(robot.getCollector().getBlockColorSensor().getValidatedColor());
                }
                // moving lift back up
                else if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                    robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_POS, Lift.StateType.TROUGH);
                    transferredOnce = true;
                }
        }
    }
    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.TROUGH;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY && transferredOnce;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.TROUGH;
    }
}
