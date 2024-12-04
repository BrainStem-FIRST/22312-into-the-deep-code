package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;
import org.firstinspires.ftc.teamcode.util.Helper;

// should only be called when lift is in trough position
public class TroughToBasketState extends RobotState<LiftingSystem.StateType> {
    public TroughToBasketState() {
        super(LiftingSystem.StateType.TROUGH_TO_BASKET);
    }
    @Override
    public void execute() {
        if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER)
            robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY);

        // can simultaneously raise lift during raising of arm if state is triggered when robot is far enough away from basket
        if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_SAFETY
        || LiftingSystem.DEPOSIT_SAFETY_DIST < Helper.dist(LiftingSystem.DEPOSIT_CORNER, robot.getDriveTrain().pose.position)) {
            // only want to set lift state to be in transition if in trough; not if its already going up
            if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
                robot.getLift().getTransitionState().setGoalState(robot.getLift().getBasketDepositPos(), Lift.StateType.BASKET_DEPOSIT);

            else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TRANSITION) {
                // if and else if below checks for basket overriding
                if(robot.isHighDeposit() && robot.getLift().getTransitionState().getGoalStatePosition() != Lift.HIGH_BASKET_POS) {
                    robot.getLift().getTransitionState().overrideGoalPosition(Lift.HIGH_BASKET_POS);
                    // robot.telemetry.addData("basket overriding during transition", "");
                }
                else if(!robot.isHighDeposit() && robot.getLift().getTransitionState().getGoalStatePosition() != Lift.LOW_BASKET_POS) {
                    robot.getLift().getTransitionState().overrideGoalPosition(Lift.LOW_BASKET_POS);
                    // robot.telemetry.addData("basket overriding during transition", "");
                }
                // optimization did not work as of 12/1; i don't know why
                // moves arm down once lift is within range of basket (not necessarily there yet)
                if(robot.getLift().getLiftMotor().getCurrentPosition() >= robot.getLift().getBasketSafetyPos()
                && robot.getLift().getLiftMotor().getCurrentPosition() <= robot.getLift().getTransitionState().getGoalStatePosition() + Lift.DESTINATION_THRESHOLD)
                    robot.getArm().getTransitionState().setGoalState(Arm.BASKET_DROP_POS, Arm.StateType.BASKET_DROP);
            }
            // lowers arm once lift reach destination
            else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.BASKET_DEPOSIT)
                robot.getArm().getTransitionState().setGoalState(Arm.BASKET_DROP_POS, Arm.StateType.BASKET_DROP);
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
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_DROP;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        // automatically drops and resets lift if button already cued; else goes to state in which waits for user input
        return robot.getLiftingSystem().getButtonACued() ? LiftingSystem.StateType.BASKET_TO_TROUGH : LiftingSystem.StateType.BASKET_DEPOSIT;
    }
}
