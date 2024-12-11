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
    public void execute(double dt) {
        if(robot.getLift().getStateManager().getActiveStateType() != Lift.StateType.TROUGH_SAFETY) {
            robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
            robot.getLift().getTransitionState().getPid().setkP(Lift.SMALL_TRANSITION_KP);
            robot.getLift().getTransitionState().getPid().setkI(Lift.SMALL_TRANSITION_KI);
            if(robot.getGrabber().hasBlock())
                robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY);
        }

        else if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DROP_OFF)
            robot.getArm().getTransitionState().overrideGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER);
    }
    @Override
    public void executeOnEntered() {
        robot.setIsDepositing(true);
        robot.getLiftingSystem().setStayInTrough(true);
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
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER || robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.BASKET_SAFETY;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.TROUGH;
    }
}
