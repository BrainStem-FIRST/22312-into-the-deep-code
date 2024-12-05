package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

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
        // accounting switching baskets after arm is already down
        if(robot.getArm().getTransitionState().getGoalStatePosition() == Arm.BASKET_DROP_POS)
            robot.getArm().getTransitionState().overrideGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY);

        // accounting switching baskets before arm is down/once arm not down
        else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_SAFETY) {
            // overriding lift to transition to high/low basket if not there and need to be there
            if (robot.isHighDeposit() && !robot.getLift().atHighBasket())
                robot.getLift().getTransitionState().overrideGoalState(Lift.HIGH_BASKET_POS, Lift.StateType.BASKET_DEPOSIT);
            else if (!robot.isHighDeposit() && !robot.getLift().atLowBasket())
                robot.getLift().getTransitionState().overrideGoalState(Lift.LOW_BASKET_POS, Lift.StateType.BASKET_DEPOSIT);

            // setting arm to lower once lift done transitioning or is close enough
            if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.BASKET_DEPOSIT
            || (robot.getLift().getLiftMotor().getCurrentPosition() >= robot.getLift().getBasketSafetyPos()
                    && robot.getLift().getLiftMotor().getCurrentPosition() <= robot.getLift().getBasketDepositPos() + Lift.DESTINATION_THRESHOLD))
                robot.getArm().getTransitionState().setGoalState(Arm.BASKET_DROP_POS, Arm.StateType.BASKET_DROP, Arm.BASKET_SAFETY_TO_BASKET_DROP_TIME);
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
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_DROP;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.BASKET_DEPOSIT;
    }
}
