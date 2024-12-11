package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
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
        // checking if grabber has specimen (means need to move lift up to clear specimen from wall)
        if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA && robot.getGrabber().hasSpecimen())
            robot.getLift().getTransitionState().setGoalState(Lift.DROP_AREA_AFTER_POS, Lift.StateType.DROP_AREA_AFTER);

        // checking if grabber does not have specimen but lift is setup as if it does (means need to lower lift so grabber can grab onto specimen)
        else if (robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA_AFTER && !robot.getGrabber().hasSpecimen()) {
            robot.getLift().getTransitionState().setGoalState(Lift.DROP_AREA_POS, Lift.StateType.DROP_AREA);
            robot.getLift().getTransitionState().getPid().setkP(Lift.SMALL_TRANSITION_KP);
        }

        // automatically transitions to trough once start collecting or if too close to basket and has block
        if (robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.SEARCH_AND_COLLECT || robot.getCollector().hasValidBlockColor())
            stateManager.tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
        // automatically transitions to specimen ram if have specimen and close enough to specimen area
        else if(robot.getGrabber().hasSpecimen())
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_RAM);
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.TROUGH_TO_DROP_AREA
                || robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.BASKET_TO_DROP_AREA;
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
