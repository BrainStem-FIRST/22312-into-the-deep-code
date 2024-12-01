package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

// TODO: fix so ends when lift goes to trought safety state, not trough (bc changd it)
// TODO: do same for ram_to_troughState
public class BasketToTroughState extends RobotState<LiftingSystem.StateType> {
    public BasketToTroughState() {
        super(LiftingSystem.StateType.BASKET_TO_TROUGH);
    }
    @Override
    public void execute() {
        // if lift still at position of basket depositing
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.BASKET_DEPOSIT) {
            // releasing block
            if(robot.getGrabber().hasBlock()) {
                robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
            }
            // resetting after grabber is open
            else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                // moving arm up if arm still at left position
                if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BLOCK_DROP)
                    robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY);
                // moving lift down when arm done rotating
                else if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_SAFETY)
                    robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
            }
        }
        // once lift reach safety threshold, move arm down
        else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {
            robot.getArm().getTransitionState().setGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER);
        }
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.BASKET_DEPOSIT;
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
