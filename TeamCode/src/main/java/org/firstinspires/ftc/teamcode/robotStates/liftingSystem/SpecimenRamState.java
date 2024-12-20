package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class SpecimenRamState extends RobotState<LiftingSystem.StateType> {
    public SpecimenRamState() {
        super(LiftingSystem.StateType.SPECIMEN_RAM);
    }
    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.DROP_AREA_TO_RAM;
    }
    @Override
    public void execute(double dt) {

        // actually ramming using lift
        if(robot.getInput().getGamepadTracker1().isFirstFrameA()
        && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_BEFORE) {
            robot.getLift().getTransitionState().setGoalState(robot.getLift().getRamAfterPos(), Lift.StateType.RAM_AFTER);
            robot.getLift().getTransitionState().getPid().setkP(Lift.BIG_TRANSITION_KP);
            robot.getLift().getTransitionState().getPid().setkI(Lift.SMALL_TRANSITION_KI);
        }
        // releasing specimen once ram is finished
        else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_AFTER) {
            robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
            robot.getGrabber().setHasSpecimen(false);
            robot.setIsDepositing(true); // resets depositing mode to true
        }
    }
    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.RAM_TO_DROP_AREA;
    }
}
