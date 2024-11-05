package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class TroughToBasketState extends RobotState<LiftingSystem.StateType> {
    public TroughToBasketState() {
        super(LiftingSystem.StateType.TROUGH_TO_BASKET);
    }

    @Override
    public void execute() {
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {

            // first moves arm up to vertical position
            if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DOWN) {
                robot.getArm().getTransitionState().setGoalState(Arm.UP_POS, Arm.StateType.UP);
            }
            // once arm done rotating up, want to move lift up to height past basket where arm can safely rotate back down
            else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.UP) {
                robot.getLift().getTransitionState().setGoalState(Lift.BASKET_SAFETY_POS, Lift.StateType.BASKET_SAFETY);
            }
        }

        // want to rotate arm back down once lift past basket and continue lift to final position
        else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.BASKET_SAFETY) {
            robot.getArm().getTransitionState().setGoalState(Arm.LEFT_POS, Arm.StateType.LEFT);
            robot.getLift().getTransitionState().setGoalState(Lift.BASKET_POS, Lift.StateType.BASKET);
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
        // arm must be in "left" position and lift must be at end position
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.LEFT &&
                robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.BASKET;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.BASKET_DEPOSIT;
    }
}
