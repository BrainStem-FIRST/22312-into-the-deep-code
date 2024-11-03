package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class BasketDepositState extends RobotState<LiftingSystem.StateType> {
    public BasketDepositState() {
        super(LiftingSystem.StateType.BASKET_DEPOSIT);
    }
    @Override
    public void execute() {
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
