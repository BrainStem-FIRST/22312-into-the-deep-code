package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class TroughToDropAreaState extends RobotState<LiftingSystem.StateType> {
    public TroughToDropAreaState() {
        super(LiftingSystem.StateType.TROUGH_TO_DROP_AREA);
    }
    @Override
    public void execute() {
        // if lift in safety pos
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
            // moving arm from down to left
            if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER)
                robot.getArm().getTransitionState().setGoalState(Arm.BLOCK_DROP_POS, Arm.StateType.BLOCK_DROP);
            // moving lift to drop area once arm is done
            else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BLOCK_DROP)
                robot.getLift().getTransitionState().setGoalState(Lift.DROP_AREA_POS, Lift.StateType.DROP_AREA);
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
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BLOCK_DROP;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.DROP_AREA;
    }
}