package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class BasketToDropAreaState extends RobotState<LiftingSystem.StateType> {
    public BasketToDropAreaState() {
        super(LiftingSystem.StateType.BASKET_TO_DROP_AREA);
    }
    @Override
    public void execute(double dt) {

        // if lift is still at/trying to reach position to deposit
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.BASKET_DEPOSIT) {
            // releasing block
            if(robot.getGrabber().hasBlock()) {
                robot.getLiftingSystem().setStayInTrough(false); // resets need to stay in trough once block is deposited
                robot.getLiftingSystem().setButtonACued(false); // resets button cuing to be set for next time
                robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
            }
            // resetting after grabber is open
            else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                // moving arm up if arm still at left position
                if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_DROP)
                    robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY, Arm.BASKET_SAFETY_TO_BASKET_DROP_TIME);
                // moving lift down when arm either passes up position or finishes rotating
                else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_SAFETY
                || robot.getArm().getTransitionState().getTime() >= Arm.BASKET_DROP_TO_UP_TIME) {
                    robot.getLift().getTransitionState().overrideGoalState(Lift.DROP_AREA_POS, Lift.StateType.DROP_AREA);
                    robot.getLift().getTransitionState().getPid().setkP(Lift.MEDIUM_TRANSITION_KP);
                }
            }
        }
        // once lift reach safety threshold, move arm down
        else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA) {
            robot.getArm().getTransitionState().setGoalState(Arm.DROP_OFF_POS, Arm.StateType.DROP_OFF, Arm.UP_TO_BASKET_SAFETY_TIME + Arm.BASKET_DROP_TO_UP_TIME);
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
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DROP_OFF
                && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.DROP_AREA;
    }
}
