package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class BasketDepositState extends RobotState<LiftingSystem.StateType> {
    public BasketDepositState() {
        super(LiftingSystem.StateType.BASKET_DEPOSIT);
    }
    @Override
    public void execute() {
        if(input.getGamepadTracker1().isBPressed())
            robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.OPENING);
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.TROUGH_TO_BASKET;
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
        return LiftingSystem.StateType.BASKET_TO_TROUGH;
    }
}
