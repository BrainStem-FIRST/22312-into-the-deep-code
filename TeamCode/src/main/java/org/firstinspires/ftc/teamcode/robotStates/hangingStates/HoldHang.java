package org.firstinspires.ftc.teamcode.robotStates.hangingStates;

import org.firstinspires.ftc.teamcode.robot.Hanger;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class HoldHang extends RobotState<Hanger.StateType> {
    public HoldHang() {
        super(Hanger.StateType.HANG_DOWN);
    }

    @Override
    public void execute(double dt) {
        robot.getHanger().setHangMotorPower(Hanger.HANG_HOLD_POWER);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Hanger.StateType.TRANSITION;
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
    public Hanger.StateType getNextStateType() {
        return null;
    }
}
