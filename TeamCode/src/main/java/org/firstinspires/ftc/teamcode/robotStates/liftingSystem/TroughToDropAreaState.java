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
        if(isFirstTime())
            robot.setIsDepositing(false);
        // if lift in safety pos
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
            // moving arm from down to left
            if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER)
                robot.getArm().getTransitionState().setGoalState(Arm.DROP_OFF_POS, Arm.StateType.DROP_OFF);
            // moving lift to drop area once arm is done
            else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DROP_OFF)
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
        return robot.getLift().getTransitionState().getGoalStatePosition() == Lift.DROP_AREA_POS;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.DROP_AREA;
    }
}