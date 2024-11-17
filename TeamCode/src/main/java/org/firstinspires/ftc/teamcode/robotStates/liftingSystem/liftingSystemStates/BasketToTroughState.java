package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Arm;
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
            // moving arm up if arm still at left position
            if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.LEFT)
                robot.getArm().getTransitionState().setGoalState(Arm.RIGHT_POS, Arm.StateType.RIGHT);
                // moving lift down once arm is set
            else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.RIGHT)
                robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
        }
        // once lift reach safety threshold
        else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {
            // moving arm down if still right
            robot.getArm().getTransitionState().setGoalState(Arm.DOWN_POS, Arm.StateType.DOWN);
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
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DOWN;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.TROUGH;
    }
}
