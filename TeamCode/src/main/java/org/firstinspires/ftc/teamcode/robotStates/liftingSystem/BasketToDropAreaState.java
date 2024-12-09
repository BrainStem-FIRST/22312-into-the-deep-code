package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class BasketToDropAreaState extends RobotState<LiftingSystem.StateType> {
    public BasketToDropAreaState() {
        super(LiftingSystem.StateType.BASKET_TO_DROP_AREA);
    }
    @Override
    public void execute() {

        // if lift is still at/trying to reach position to deposit
        if(robot.getLift().getTransitionState().getGoalStatePosition() == robot.getLift().getBasketDepositPos()) {
            // releasing block
            if(robot.getGrabber().hasBlock()) {
                robot.getLiftingSystem().setStayInTrough(false); // resets need to stay in trough once block is deposited
                robot.getLiftingSystem().setButtonACued(false); // resets button cuing to be set for next time
                robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
            }
            // resetting after grabber is open
            else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                // moving arm up if arm still at left position
                if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_DROP)
                    robot.getArm().getTransitionState().setGoalState(Arm.DROP_OFF_POS, Arm.StateType.DROP_OFF, Arm.BASKET_DROP_TO_DROP_OFF_TIME);
                // moving lift down when arm either passes up position or finishes rotating
                else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_DROP
                || robot.getArm().getTransitionState().getTime() >= Arm.BASKET_DROP_TO_UP_TIME) {
                    robot.getLift().getTransitionState().overrideGoalState(Lift.DROP_AREA_POS, Lift.StateType.DROP_AREA);
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
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DROP_OFF
                && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.DROP_AREA;
    }
}
