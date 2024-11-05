package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class SpecimenRamState extends RobotState<LiftingSystem.StateType> {
    public SpecimenRamState() {
        super(LiftingSystem.StateType.SPECIMEN_RAM);
    }
    @Override
    public void execute() {
        if(input.getGamepadTracker1().isBPressed())
            robot.getLift().getTransitionState().setGoalState(Lift.RAM_AFTER_POS, Lift.StateType.RAM_AFTER);
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_AFTER) {
            robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.OPENING);
            robot.getGrabber().setHasSpecimen(false);
        }
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.DROP_AREA_TO_RAM;
    }

    @Override
    public boolean canBeOverridden() {
        // lift and grabber must both be static
        return robot.getLift().getStateManager().getActiveStateType() != Lift.StateType.TRANSITION &&
                robot.getGrabber().getStateManager().getActiveStateType() != Grabber.StateType.OPENING;
    }

    @Override
    public boolean isDone() {
        return robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.RAM_TO_TROUGH;
    }
}
