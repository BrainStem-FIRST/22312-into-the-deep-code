package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;


public class TroughState extends RobotState<LiftingSystem.StateType> {
    public TroughState() {
        super(LiftingSystem.StateType.TROUGH);
    }
    @Override
    public void execute() {
        if(!robot.canTransfer()) {
            // handling the resetting of lift if suddenly cannot transfer
            if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TRANSITION)
                robot.getLift().getTransitionState().overrideGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
            else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH)
                robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
            // handling resetting of grabber
            if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED)
                robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
            else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.TRANSITION)
                robot.getGrabber().getTransitionState().overrideGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
        }
        // checking if extension properly set
        else if(robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.IN && robot.getCollector().hasValidBlockColor()) {
            // opening grabber and lowering lift if at trough safety
            if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY) {
                // need to open grabber each time bc if have failed transfer grabber stays closed (also why we reset hasBlock to false)
                if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                    robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                    robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
                }
                else if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN)
                    robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_POS, Lift.StateType.TROUGH);
            }
            // closing onto block once lift is down
            else if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH) {
                robot.getGrabber().getTransitionState().setGoalState(Grabber.CLOSE_POS, Grabber.StateType.CLOSED);
                // waiting for grabber to close on block before raising lift
                if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                    robot.getGrabber().setBlockColorHeld(robot.getCollector().getBlockColorInTrough());
                    robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
                }
            }
        }
    }

    @Override
    public boolean canEnter() {
        // lift must properly be lowered and set before entering
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.BASKET_TO_TROUGH ||
                robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.RAM_TO_TROUGH;
    }

    // only can be overridden if lift is at trough safety and if grabber has block
    @Override
    public boolean canBeOverridden() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY && robot.getGrabber().hasBlock();
    }

    @Override
    public boolean isDone() {
        // want to automatically transition to drop area state if block color held is equal to alliance color; only time when this function should evaluate to true
        return robot.getGrabber().getBlockColorHeld() == robot.getColorFromAlliance()
            && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        // always want to transition to drop area bc if block is yellow then isDone will never be true and this state will be overridden when gamepad1 presses a
        return LiftingSystem.StateType.TROUGH_TO_DROP_AREA;
    }
}
