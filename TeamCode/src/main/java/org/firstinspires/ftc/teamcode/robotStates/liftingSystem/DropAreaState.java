package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class DropAreaState extends RobotState<LiftingSystem.StateType> {
    public DropAreaState() {
        super(LiftingSystem.StateType.DROP_AREA);
    }
    @Override
    public void execute(double dt) {
        // checking input
        if (robot.getInput().getGamepadTracker1().isFirstFrameA()) {
            if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED)
                // if doesn't have specimen (means has block) then open grabber to drop off block
                if (!robot.getGrabber().hasSpecimen()) {
                    robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                    robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
                }
                // if has specimen then proceed to prep for ram
                else
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_RAM);
            // grabbing specimen if grabber is open and moving arm to clear specimen off wall
            else if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                robot.getGrabber().getTransitionState().overrideGoalState(Grabber.CLOSE_POS, Grabber.StateType.CLOSED);
                robot.getGrabber().setHasSpecimen(true);
                robot.getArm().getTransitionState().overrideGoalState(Arm.DROP_OFF_AFTER_POS, Arm.StateType.DROP_OFF_AFTER);
            }
        }

        // if grabber is closing or already closed, then open grabber (should run when fails to grab specimen)
        if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
            // transitioning to trough for deposit bc u have block
            if(robot.getInput().getGamepadTracker2().isFirstFrameB() && !robot.getGrabber().hasSpecimen())
                robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
            // in case driver misses specimen and closes grabber; needs way to reset
            else if (robot.getInput().getGamepadTracker1().isFirstFrameB() && robot.getGrabber().hasSpecimen()) {
                robot.getGrabber().getTransitionState().overrideGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
                robot.getArm().getTransitionState().overrideGoalState(Arm.DROP_OFF_POS, Arm.StateType.DROP_OFF);
            }
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
