package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class RamToDropAreaState extends RobotState<LiftingSystem.StateType> {
    public RamToDropAreaState() {
        super(LiftingSystem.StateType.RAM_TO_DROP_AREA);
    }
    @Override
    public void execute() {
        robot.getArm().getTransitionState().setGoalState(Arm.DROP_OFF_POS, Arm.StateType.DROP_OFF);
        robot.getLift().getTransitionState().setGoalState(Lift.DROP_AREA_POS, Lift.StateType.DROP_AREA);
        robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.DROP_AREA_TO_RAM ||
                robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.SPECIMEN_RAM;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA &&
                robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DROP_OFF &&
                robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.DROP_AREA;
    }
}
