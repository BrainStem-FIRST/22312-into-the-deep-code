package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class DropAreaToRamState extends RobotState<LiftingSystem.StateType> {
    public DropAreaToRamState() {
        super(LiftingSystem.StateType.DROP_AREA_TO_RAM);
    }
    @Override
    public void execute() {
        robot.getArm().getTransitionState().setGoalState(Arm.RIGHT_POS, Arm.StateType.RIGHT);
        robot.getLift().getTransitionState().setGoalState(Lift.RAM_BEFORE_POS, Lift.StateType.RAM_BEFORE);
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.DROP_AREA;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        // done when arm along hard stop and lift in position below bar
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.RIGHT &&
                robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_BEFORE;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.SPECIMEN_RAM;
    }
}
