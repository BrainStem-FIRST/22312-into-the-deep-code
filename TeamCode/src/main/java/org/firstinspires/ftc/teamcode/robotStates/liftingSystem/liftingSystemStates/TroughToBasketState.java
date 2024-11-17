package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

// should only be called when lift is in trough position
public class TroughToBasketState extends RobotState<LiftingSystem.StateType> {
    public TroughToBasketState() {
        super(LiftingSystem.StateType.TROUGH_TO_BASKET);
    }
    @Override
    public void execute() {
        if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DOWN)
            robot.getArm().getTransitionState().setGoalState(Arm.RIGHT_POS, Arm.StateType.RIGHT);

        else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.RIGHT) {
            // only want to set lift state to be in transition if in trough
            if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
                if (robot.isHighDeposit())
                    robot.getLift().getTransitionState().setGoalState(Lift.HIGH_BASKET_POS, Lift.StateType.BASKET_DEPOSIT);
                else
                    robot.getLift().getTransitionState().setGoalState(Lift.LOW_BASKET_POS, Lift.StateType.BASKET_DEPOSIT);

            // checking if when lift is in transition gamepad 2 switches deposit state
            else if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TRANSITION) {
                if (robot.isHighDeposit() && robot.getLift().getTransitionState().getGoalStatePosition() != Lift.HIGH_BASKET_POS) {
                    robot.getLift().getTransitionState().overrideGoalState(Lift.HIGH_BASKET_POS);
                    robot.telemetry.addData("basket overriding during transition", "");
                } else if (!robot.isHighDeposit() && robot.getLift().getTransitionState().getGoalStatePosition() != Lift.LOW_BASKET_POS) {
                    robot.getLift().getTransitionState().overrideGoalState(Lift.LOW_BASKET_POS);
                    robot.telemetry.addData("basket overriding during transition", "");
                }
            }

            // moving arm down after lift reach desired basket height
            else if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.BASKET_DEPOSIT)
                robot.getArm().getTransitionState().setGoalState(Arm.LEFT_POS, Arm.StateType.LEFT);
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
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.LEFT;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.BASKET_DEPOSIT;
    }
}
