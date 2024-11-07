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
        robot.getArm().getStateManager().tryEnterState(Arm.StateType.RIGHT);
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA)
            if(robot.isHighRam())
                robot.getLift().getTransitionState().setGoalState(Lift.HIGH_RAM_BEFORE_POS, Lift.StateType.RAM_BEFORE);
            else
                robot.getLift().getTransitionState().setGoalState(Lift.LOW_RAM_BEFORE_POS, Lift.StateType.RAM_BEFORE);
    }

    @Override
    public boolean canEnter() {
        return false;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return null;
    }
}
