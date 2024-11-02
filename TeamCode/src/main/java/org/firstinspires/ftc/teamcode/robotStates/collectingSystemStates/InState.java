package org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robotStates.RobotStateTele;
import org.firstinspires.ftc.teamcode.tele.CollectingSystemTele;

public class InState extends CollectSystemState {
    public InState() {
        super(CollectingSystemTele.StateType.IN);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean canEnter() {
        return collectingSystemTele.getStateManager().getActiveStateType() == CollectingSystemTele.StateType.RETRACTING;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    // waits for other states to override this
    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public CollectingSystemTele.StateType getNextStateType() {
        return CollectingSystemTele.StateType.IN;
    }
}
