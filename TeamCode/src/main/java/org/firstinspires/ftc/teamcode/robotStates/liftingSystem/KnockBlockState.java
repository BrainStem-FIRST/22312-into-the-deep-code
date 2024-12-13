package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class KnockBlockState extends RobotState<LiftingSystem.StateType> {
    private boolean knockedYet;
    public KnockBlockState() {
        super(LiftingSystem.StateType.KNOCK_BLOCK);
        knockedYet = false;
    }
    @Override
    public void executeOnEntered() {
        knockedYet = false;
    }
    @Override
    public void execute(double dt) {
        // prepping for knocking
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {
            robot.getArm().getTransitionState().setGoalState(Arm.KNOCK_BLOCK_POS, Arm.StateType.KNOCK_BLOCK, Arm.TRANSFER_TO_KNOCK_BLOCK_TIME);
            robot.getLift().getTransitionState().setGoalState(Lift.KNOCK_BLOCK_POS, Lift.StateType.KNOCK_BLOCK);
            robot.getLift().getTransitionState().getPid().setkI(Lift.SMALL_TRANSITION_KI);
        }
        // actually knocking and then resetting
        else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.KNOCK_BLOCK) {
            if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.KNOCK_BLOCK)
                robot.getArm().getTransitionState().setGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER, Arm.TRANSFER_TO_KNOCK_BLOCK_TIME);
            else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER) {
                knockedYet = true;
                robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
                robot.getLift().getTransitionState().getPid().setkI(Lift.SMALL_TRANSITION_KI);
            }
        }

    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.TROUGH
                && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY
                && robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY
                && knockedYet;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.TROUGH;
    }
}
