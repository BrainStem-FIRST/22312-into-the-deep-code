package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class DropAreaToRamState extends RobotState<LiftingSystem.StateType> {
    public DropAreaToRamState() {
        super(LiftingSystem.StateType.DROP_AREA_TO_RAM);
    }

    @Override
    public void execute(double dt) {
        if (isFirstTime()) {
            robot.getLift().getTransitionState().setGoalState(robot.getLift().getRamBeforePos(), Lift.StateType.RAM_BEFORE);
            robot.getLift().getTransitionState().getPid().setkP(Lift.MEDIUM_TRANSITION_KP);
            robot.getArm().getTransitionState().setGoalState(Arm.SPECIMEN_HANG_POS, Arm.StateType.SPECIMEN_HANG, Arm.DROP_AREA_TO_RAM_TIME);
        }
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
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.SPECIMEN_HANG;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.SPECIMEN_RAM;
    }
}
