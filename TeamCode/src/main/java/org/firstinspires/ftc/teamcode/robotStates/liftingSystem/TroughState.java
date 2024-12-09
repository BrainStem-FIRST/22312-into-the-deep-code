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
            // handling the resetting of lift to trough safety
            if(robot.getLift().getTransitionState().getGoalStatePosition() == Lift.TROUGH_POS)
                robot.getLift().getTransitionState().overrideGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);

            // handling resetting of grabber
            if(robot.getGrabber().getTransitionState().getGoalStatePosition() == Grabber.CLOSE_POS)
                robot.getGrabber().getTransitionState().overrideGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
        }

        // handling actual transfer
        else if(robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.IN && robot.getCollector().hasValidBlockColor()) {
            // transfer stage 1: opening grabber, setting arm to transfer pos, and lowering lift if at trough safety
            if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {
                // resetting grabber to be ready to transfer
                if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                    robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                    robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
                }
                // moving arm into position to transfer
                if(robot.getArm().getStateManager().getActiveStateType() != Arm.StateType.TRANSFER)
                    robot.getArm().getTransitionState().overrideGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER);

                // lowering lift to transfer once both subsystems are ready
                if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN
                && robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER) {
                    robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_POS, Lift.StateType.TROUGH);
                    robot.getLift().getTransitionState().getPid().setkI(Lift.SMALL_TRANSITION_KI);
                }
            }
            // transfer stage 2: closing onto block once lift is down and raising lift
            else if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH) {
                if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                    robot.getGrabber().getTransitionState().setGoalState(Grabber.CLOSE_POS, Grabber.StateType.CLOSED);
                    robot.getGrabber().setBlockColorHeld(robot.getCollector().getBlockColorInTrough());
                    robot.setIsDepositing(robot.getGrabber().getBlockColorHeld() == BlockColor.YELLOW); // here I set isDepositing automatically (basically, it is reset every time transfer occurs)
                    robot.getLiftingSystem().setStayInTrough(false); // I also reset stayInTrough every time there is a transfer
                }
                // waiting for grabber to close on block before raising lift
                if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                    robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
                }
            }
        }

        // handling what to do once have successful transfer
        else if (!robot.getCollector().hasValidBlockColor()
        && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
            // prepping for basket deposit
            if (robot.getGrabber().hasBlock() && robot.isDepositing())
                robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY, Arm.TRANSFER_TO_BASKET_SAFETY_TIME);
            // prepping for specimen pickup at human player station
            else if (!robot.getLiftingSystem().getStayInTrough())
                robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_DROP_AREA);
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
        return false;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return null;
    }
}
