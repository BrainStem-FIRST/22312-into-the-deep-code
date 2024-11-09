package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class BasketToTroughState extends RobotState<LiftingSystem.StateType> {
    public BasketToTroughState() {
        super(LiftingSystem.StateType.BASKET_TO_TROUGH);
    }
    @Override
    public void execute() {
        robot.telemetry.addData("inside basketToTroughState's execute function", "");
        // TODO: during testing, if arm servo is fast enough, can make lift lower when arm is raising to clear basket AND when arm lowering to down position
        // if lift still at position of basket depositing
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.BASKET_DEPOSIT) {
            // moving arm up if arm still at left position
            if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.LEFT)
                robot.getArm().getTransitionState().setGoalState(Arm.UP_POS, Arm.StateType.UP);
                // moving lift down once arm is up
            else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.UP)
                robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
        }
        // once lift reach safety threshold
        else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {
            robot.telemetry.addData("lift state recognized as trough safety inside basketToTrough execute function", "");
            // moving arm down if still up
            if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.UP) {
                robot.getArm().getTransitionState().setGoalState(Arm.DOWN_POS, Arm.StateType.DOWN);
            }
                // moving lift fully down once arm is down
            else if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DOWN) {
                robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_POS, Lift.StateType.TROUGH);
            }
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
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.TROUGH;
    }
}