package org.firstinspires.ftc.teamcode.robotStates.extensionStates;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class InState extends RobotState<Extension.StateType> {
    public InState() {
        super(Extension.StateType.IN);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Extension.StateType.RETRACTING;
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
    public Extension.StateType getNextStateType() {
        return Extension.StateType.IN;
    }
}
