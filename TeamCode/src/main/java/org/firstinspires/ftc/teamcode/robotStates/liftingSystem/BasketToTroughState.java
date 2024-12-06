package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class BasketToTroughState extends RobotState<LiftingSystem.StateType> {
    public BasketToTroughState() {
        super(LiftingSystem.StateType.BASKET_TO_TROUGH);
    }
    @Override
    public void execute() {

        // if lift is still at/trying to reach position to deposit
        if(robot.getLift().getTransitionState().getGoalStatePosition() == robot.getLift().getBasketDepositPos()) {
        //OLD CODE (but it works): if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.BASKET_DEPOSIT) {
            // releasing block
            if(robot.getGrabber().hasBlock()) {
                robot.getLiftingSystem().setButtonACued(false); // resets button cuing to be set for next time
                robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
            }
            // resetting after grabber is open
            else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                // moving arm up if arm still at left position
                if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_DROP)
                    robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY, Arm.BASKET_SAFETY_TO_BASKET_DROP_TIME);
                // moving lift down when arm either passes up position or finishes rotating
                else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_SAFETY
                || robot.getArm().getTransitionState().getTime() >= Arm.BASKET_DROP_TO_UP_TIME) {
                    robot.getLift().getTransitionState().overrideGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
                    robot.getLift().getTransitionState().getPid().setkP(Lift.LOWERING_KP);
                    robot.getLift().getTransitionState().getPid().setkI(0);
                    robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY, Arm.UP_TO_BASKET_SAFETY_TIME);
                }
            }
        }
        // once lift reach safety threshold, move arm down
        else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {
            robot.getArm().getTransitionState().setGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER, Arm.TRANSFER_TO_BASKET_SAFETY_TIME);
        }
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.BASKET_DEPOSIT
                || robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.TROUGH_TO_BASKET;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.TROUGH;
    }
}
