package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;
import org.firstinspires.ftc.teamcode.util.Helper;

public class BasketResettingState extends RobotState<LiftingSystem.StateType> {
    public BasketResettingState() {
        super(LiftingSystem.StateType.BASKET_RESETTING);
    }
    @Override
    public void execute() {
        // releasing block
        if(robot.getGrabber().hasBlock()) {
            robot.getLiftingSystem().setButtonACued(false); // resets button cuing to be set for next time
            robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
            robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
        }
        // resetting after grabber is open
        else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
            // moving arm up if arm still at left position
            if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_DROP)
                robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY, Arm.BASKET_SAFETY_TO_BASKET_DROP_TIME);

            // checking when to reset lift and arm
            else if(robot.getArm().getTransitionState().getTime() >= Arm.BASKET_DROP_TO_UP_TIME
            || robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_SAFETY) {
                // resetting arm
                if (Helper.dist(robot.getDriveTrain().pose.position, LiftingSystem.DEPOSIT_SAFETY_POS) >= LiftingSystem.DEPOSIT_SAFETY_DIST)
                    robot.getArm().getTransitionState().overrideGoalState(Arm.DROP_OFF_POS, Arm.StateType.DROP_OFF);
                // lowering lift and resetting arm if optimization does not work
                if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.BASKET_DEPOSIT)
                    robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
                else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {
                    robot.getLift().getTransitionState().setGoalStateWithoutPid(Lift.DROP_AREA_POS, Lift.StateType.DROP_AREA);
                    robot.getArm().getTransitionState().overrideGoalState(Arm.DROP_OFF_POS, Arm.StateType.DROP_OFF);
                }
            }
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
        // checking for both is done for when transitioning to drop area and trough
        return (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DROP_OFF && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA);
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.DROP_AREA;
        //return robot.getLiftingSystem().isResetToTrough() ? LiftingSystem.StateType.TROUGH : LiftingSystem.StateType.DROP_AREA;
    }
}
