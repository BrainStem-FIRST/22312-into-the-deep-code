package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;
import org.firstinspires.ftc.teamcode.util.Helper;

public class DropAreaState extends RobotState<LiftingSystem.StateType> {
    public DropAreaState() {
        super(LiftingSystem.StateType.DROP_AREA);
    }
    @Override
    public void execute(double dt) {
        if (robot.getInput().getGamepadTracker1().isFirstFrameA()) {
            // what to do if grabber is closed
            if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED)
                // if doesn't have specimen (means has block) then open grabber to drop off block
                if (!robot.getGrabber().hasSpecimen()) {
                    robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                    robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
                }
                // if has specimen then proceed to prep for ram
                else
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_RAM);
            // grabbing specimen if grabber is open and moving lift to clear specimen off wall
            else if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                robot.getGrabber().getTransitionState().overrideGoalState(Grabber.CLOSE_POS, Grabber.StateType.CLOSED);
                robot.getGrabber().setHasSpecimen(true);
                robot.getLift().getTransitionState().overrideGoalState(Lift.DROP_AREA_AFTER_POS, Lift.StateType.DROP_AREA_AFTER);
                robot.getLift().getTransitionState().getPid().setkI(Lift.SMALL_TRANSITION_KI);
                robot.getLift().getTransitionState().getPid().setkP(Lift.MEDIUM_TRANSITION_KP);
            }
        }

        // if grabber is closing or already closed, then open grabber (should run when fails to grab specimen)
        if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
            // transitioning to trough for deposit bc u have block
            if(!robot.getGrabber().hasSpecimen() && robot.getInput().getGamepadTracker2().isFirstFrameB())
                robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
            // in case driver misses specimen and closes grabber; needs way to reset
            else if (robot.getGrabber().hasSpecimen() && robot.getInput().getGamepadTracker1().isFirstFrameB()) {
                robot.getGrabber().getTransitionState().overrideGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
                robot.getLift().getTransitionState().overrideGoalState(Lift.DROP_AREA_POS, Lift.StateType.DROP_AREA);
                robot.getLift().getTransitionState().getPid().setkI(Lift.SMALL_TRANSITION_KI);
            }
        }
        // resetting lifting system to trough for transfer again
        else if ((robot.getInput().getGamepadTracker2().isFirstFrameB() || robot.getInput().getGamepadTracker1().isFirstFrameB())
                && robot.getGrabber().getTransitionState().getGoalStatePosition() == Grabber.OPEN_POS) {
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
        }

        // automatically transitions to trough once start collecting or if block detected in trough
        if (robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.SEARCH_AND_COLLECT || robot.getCollector().hasValidBlockColor())
            stateManager.tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.TROUGH_TO_DROP_AREA
                || robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.BASKET_TO_DROP_AREA
                || robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.RAM_TO_DROP_AREA;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return false;
    }

    // shouldn't need to return anything because isDone always returns null
    @Override
    public LiftingSystem.StateType getNextStateType() {
        return null;
    }
}
