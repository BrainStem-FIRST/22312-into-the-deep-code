package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class DropAreaToTroughState extends RobotState<LiftingSystem.StateType> {
    public DropAreaToTroughState() {
        super(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
    }
    @Override
    public void execute() {
        if(isFirstTime())
            robot.setIsDepositing(true);
        // note: i use overrideGoalState here in case this state is entered while lift/arm is in transition (to make it quicker)
        if(robot.getLift().getStateManager().getActiveStateType() != Lift.StateType.TROUGH_SAFETY)
            robot.getLift().getTransitionState().overrideGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);

        else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
            robot.getArm().getTransitionState().overrideGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER);
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.DROP_AREA;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.TROUGH;
    }
}
