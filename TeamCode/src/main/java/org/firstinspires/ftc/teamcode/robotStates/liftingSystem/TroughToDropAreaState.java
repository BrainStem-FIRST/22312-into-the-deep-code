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
    public void executeOnEntered() {
        robot.setIsDepositing(false);
        robot.getLiftingSystem().setStayInTrough(false);

        robot.getArm().getTransitionState().setGoalState(Arm.DROP_OFF_POS, Arm.StateType.DROP_OFF, Arm.TRANSFER_TO_DROP_AREA_TIME);
        robot.getLift().getTransitionState().setGoalState(Lift.DROP_AREA_POS, Lift.StateType.DROP_AREA);
        robot.getLift().getTransitionState().getPid().setkP(Lift.MEDIUM_TRANSITION_KP);
        robot.getLift().getTransitionState().getPid().setkI(Lift.SMALL_TRANSITION_KI);

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