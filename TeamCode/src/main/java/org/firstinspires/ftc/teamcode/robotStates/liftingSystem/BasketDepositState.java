package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class BasketDepositState extends RobotState<LiftingSystem.StateType> {
    public BasketDepositState() {
        super(LiftingSystem.StateType.BASKET_DEPOSIT);
    }
    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.TROUGH_TO_BASKET
                || robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.BASKET_TO_BASKET;
    }
    @Override
    public void execute(double dt) {
        // opening grabber
        if(robot.getInput().getGamepadTracker2().isFirstFrameA()) {
            robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
            robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
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
        return LiftingSystem.StateType.BASKET_TO_DROP_AREA;
    }
}
