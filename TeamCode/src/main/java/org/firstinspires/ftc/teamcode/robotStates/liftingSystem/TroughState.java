package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;


public class TroughState extends RobotState<LiftingSystem.StateType> {
    public TroughState() {
        super(LiftingSystem.StateType.TROUGH);
    }
    @Override
    public void execute() {
        // handling override of transfer if suddenly cannot transfer
        if(!robot.canTransfer()) {
            // handling the resetting of lift if suddenly cannot transfer
            if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TRANSITION)
                robot.getLift().getTransitionState().overrideGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
            else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH)
                robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);

            // would want to continue transfer if and only if block has been latched onto (would not want to release grabber then)
            else if(!robot.getCollector().hasValidBlockColor() && robot.getLift().getTransitionState().getGoalStatePosition() == Lift.TROUGH_SAFETY_POS)
                return;
            // handling resetting of grabber
            if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED)
                robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
            else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.TRANSITION)
                robot.getGrabber().getTransitionState().overrideGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
        }
        // handling actual transfer
        else if(robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.IN && robot.getCollector().hasValidBlockColor()) {
            // opening grabber and lowering lift if at trough safety
            if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {
                // need to open grabber each time bc if have failed transfer grabber stays closed (also why we reset hasBlock to false)
                if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                    robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                    robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
                }
                // lowering lift once grabber is open
                if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                    robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_POS, Lift.StateType.TROUGH);
                }
            }
            // closing onto block once lift is down
            else if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH) {
                if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                    robot.getGrabber().getTransitionState().setGoalState(Grabber.CLOSE_POS, Grabber.StateType.CLOSED);
                    robot.getGrabber().setBlockColorHeld(robot.getCollector().getBlockColorInTrough());
                    // here I set isDepositing automatically (basically, it is reset every time transfer occurs)
                    robot.setIsDepositing(robot.getGrabber().getBlockColorHeld() == BlockColor.YELLOW);
                }
                // waiting for grabber to close on block before raising lift
                if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                    robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
                }
            }
        }
        else if(!robot.getCollector().hasValidBlockColor()
                && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
            if(robot.getGrabber().hasBlock()
                && robot.isDepositing())
                robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY, Arm.TRANSFER_TO_BASKET_SAFETY_TIME);
            else if(!robot.getLiftingSystem().getStayInTrough())
                robot.setIsDepositing(false);
    }

    @Override
    public boolean canEnter() {
        // lift must properly be lowered and set before entering
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.BASKET_TO_TROUGH
                || robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.DROP_AREA_TO_TROUGH
                || robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.RAM_TO_TROUGH;
    }

    // only can be overridden if lift is at trough safety and if grabber has block
    @Override
    public boolean canBeOverridden() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY;
    }

    @Override
    public boolean isDone() {
        // want to automatically transition to drop area state if not depositing; only time when this function should evaluate to true
        return !robot.getCollector().hasValidBlockColor()
                && !robot.isDepositing()
                && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        // always want to transition to drop area bc if block is yellow then isDone will never be true and this state will be overridden when gamepad1 presses a
        return LiftingSystem.StateType.TROUGH_TO_DROP_AREA;
    }
}
