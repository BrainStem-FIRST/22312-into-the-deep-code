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
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA)
            robot.getLift().getTransitionState().setGoalState(robot.getLift().getRamBeforePos(), Lift.StateType.RAM_BEFORE);
        //else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_BEFORE)
        else if(robot.getLift().getLiftMotor().getCurrentPosition() >= Lift.TROUGH_SAFETY_POS)
            robot.getArm().getTransitionState().setGoalState(Arm.SPECIMEN_HANG_POS, Arm.StateType.SPECIMEN_HANG);
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.DROP_AREA ||
                robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.RAM_TO_DROP_AREA;
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
