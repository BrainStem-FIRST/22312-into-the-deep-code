package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.BlockColorSensor;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

import kotlin.contracts.Returns;

public class TroughState extends RobotState<LiftingSystem.StateType> {
    public TroughState() {
        super(LiftingSystem.StateType.TROUGH);
    }
    @Override
    public void execute() {
        // waiting for collection system to finish in taking block before grabbing onto it
        if(robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.IN &&
                robot.getCollector().hasValidBlockColor()) {
            robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.CLOSING);
        }
        // waiting for grabber to close on block before raising lift
        else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
            robot.getGrabber().setHasBlock(true); 
            robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
        }
    }

    @Override
    public boolean canEnter() {
        // lift must properly be lowered and set before entering
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.BASKET_TO_TROUGH ||
                robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.RAM_TO_TROUGH;
    }

    // only can be overridden if grabber is not closing in on block
    @Override
    public boolean canBeOverridden() {
        return robot.getGrabber().getStateManager().getActiveStateType() != Grabber.StateType.CLOSING;
    }

    @Override
    public boolean isDone() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        // if block is yellow, transition to basket, and if block is alliance color then transition to drop off to human player
        if(robot.getBlockColorHeld() == BlockColor.YELLOW)
            return LiftingSystem.StateType.TROUGH_TO_BASKET;
        else
            return LiftingSystem.StateType.TROUGH_TO_DROP_AREA;
    }
}
