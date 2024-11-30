package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class BasketToBasketState extends RobotState<LiftingSystem.StateType> {
    public BasketToBasketState() {
        super(LiftingSystem.StateType.BASKET_TO_BASKET);
    }
    @Override
    public void execute() {
        robot.telemetry.addData("inside execute of basket to basket state of lifting system", "");
        // accounting switching baskets after arm is already down
        if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BLOCK_DROP)
            robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY);

        // accounting switching baskets before arm is down/once arm not down
        else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_SAFETY) {

            // setting lift to transition to high basket if not there and need to be there
            if (robot.isHighDeposit() && !robot.getLift().atHighBasket())
                    robot.getLift().getTransitionState().setGoalState(Lift.HIGH_BASKET_POS, Lift.StateType.BASKET_DEPOSIT);

            else if (!robot.isHighDeposit() && !robot.getLift().atLowBasket())
                robot.getLift().getTransitionState().setGoalState(Lift.LOW_BASKET_POS, Lift.StateType.BASKET_DEPOSIT);

            // setting arm to lower once lift done transitioning (should make sense bc this conditional is only checked if arm is up
            if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.BASKET_DEPOSIT)
                robot.getArm().getTransitionState().setGoalState(Arm.BLOCK_DROP_POS, Arm.StateType.BLOCK_DROP);
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
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BLOCK_DROP &&
                ((robot.isHighDeposit() && robot.getLift().atHighBasket()) || (!robot.isHighDeposit() && robot.getLift().atLowBasket()));
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.BASKET_DEPOSIT;
    }
}
