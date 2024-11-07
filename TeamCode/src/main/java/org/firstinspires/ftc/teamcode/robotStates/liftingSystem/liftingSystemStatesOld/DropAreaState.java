package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class DropAreaState extends RobotState<LiftingSystem.StateType> {
    public DropAreaState() {
        super(LiftingSystem.StateType.DROP_AREA);
    }
    @Override
    public void execute() {
        // TODO: ask if possible to automatically drop block as soon as state is entered
        if(input.getGamepadTracker1().isBPressed())
            // to drop off block for human player to attach clip
            if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.OPENING);
                robot.getGrabber().setHasBlock(false);
            }
            // to pick up specimen human player has created
            else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.CLOSING);
                robot.getGrabber().setHasSpecimen(true);
            }
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.TROUGH_TO_DROP_AREA;
    }

    @Override
    public boolean canBeOverridden() {
        // can only be overridden if grabber is not in process of dropping/picking up block
        return robot.getGrabber().getStateManager().getActiveStateType() != Grabber.StateType.OPENING &&
                robot.getGrabber().getStateManager().getActiveStateType() != Grabber.StateType.CLOSING;
    }

    @Override
    public boolean isDone() {
        return robot.getGrabber().getHasSpecimen() && robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.DROP_AREA_TO_RAM;
    }
}
