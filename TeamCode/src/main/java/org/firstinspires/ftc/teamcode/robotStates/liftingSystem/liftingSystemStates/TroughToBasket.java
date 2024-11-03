package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class TroughToBasket extends RobotState<LiftingSystem.StateType> {
    public TroughToBasket() {
        super(LiftingSystem.StateType.TROUGH_TO_BASKET);
    }

    @Override
    public void execute() {
        if(robot.getLift().getLiftMotor().getCurrentPosition() < Lift.TROUGH_SAFETY_POSITION)
            robot.getLift().get
            robot.getLift().getStateManager().tryEnterState(Lift.StateType.TRANSITION);
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
