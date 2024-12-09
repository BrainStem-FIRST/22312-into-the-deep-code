package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

// Note: only transfers once and then returns true
public class TransferState extends RobotState<LiftingSystem.StateType>{
    public TransferState() {
        super(LiftingSystem.StateType.TRANSFER);
    }
    @Override
    public void execute() {
        // transfer stage 1: opening grabber, setting arm to transfer pos, and lowering lift if at trough safety
        if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {
            // resetting grabber to be ready to transfer
            if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
            }
            // moving arm into position to transfer
            if (robot.getArm().getStateManager().getActiveStateType() != Arm.StateType.TRANSFER)
                robot.getArm().getTransitionState().overrideGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER);

            // lowering lift to transfer once both subsystems are ready
            if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN
                    && robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER) {
                robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_POS, Lift.StateType.TROUGH);
                robot.getLift().getTransitionState().getPid().setkI(Lift.SMALL_TRANSITION_KI);
            }
        }
        // transfer stage 2: closing onto block once lift is down and ending transfer
        else if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH) {
            if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                robot.getGrabber().setBlockColorHeld(robot.getCollector().getBlockColorInTrough());
                robot.setIsDepositing(robot.getGrabber().getBlockColorHeld() == BlockColor.YELLOW); // here I set isDepositing automatically (basically, it is reset every time transfer occurs)
                robot.getLiftingSystem().setStayInTrough(false); // I also reset stayInTrough every time there is a transfer

                robot.getGrabber().getTransitionState().setGoalState(Grabber.CLOSE_POS, Grabber.StateType.CLOSED);
            }
            // waiting for grabber to close on block before resetting transfer
            if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                robot.setShouldTransfer(false);
            }
        }
        // transfer stage 3 and sudden overriding: resetting lift to safety position if suddenly cannot transfer
        if(!robot.shouldTransfer())
            // moves lift up if not already going there
            if(robot.getLift().getTransitionState().getGoalStatePosition() != Lift.TROUGH_SAFETY_POS)
                robot.getLift().getTransitionState().overrideGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);

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
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY
                && robot.getGrabber().hasBlock();
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.TROUGH;
    }
}
