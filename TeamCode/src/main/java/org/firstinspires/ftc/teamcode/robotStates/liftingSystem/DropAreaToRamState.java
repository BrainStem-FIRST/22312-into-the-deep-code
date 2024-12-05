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
    public void execute() {
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA_AFTER) {
            robot.getLift().getTransitionState().setGoalState(robot.getLift().getRamBeforePos(), Lift.StateType.RAM_BEFORE);
            robot.getArm().getTransitionState().setGoalState(Arm.SPECIMEN_RAM_POS, Arm.StateType.SPECIMEN_RAM, Arm.DROP_OFF_TO_SPECIMEN_RAM_TIME);
        }
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.DROP_AREA
                && robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.SPECIMEN_RAM;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.SPECIMEN_RAM;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.SPECIMEN_RAM;
    }
}
