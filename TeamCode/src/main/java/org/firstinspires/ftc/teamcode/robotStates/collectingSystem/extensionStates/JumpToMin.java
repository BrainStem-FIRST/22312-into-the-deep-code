package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class JumpToMin extends RobotState<Extension.StateType> {

    public static int THRESHOLD = 2;

    public JumpToMin() {
        super(Extension.StateType.JUMP_TO_MIN);
    }

    @Override
    public void execute(double dt) {
        // go to min position
        robot.getExtension().setExtensionMotorPower(Extension.JUMP_TO_MIN_POWER);
    }

    @Override
    public boolean canEnter() {
        return robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.IN
                || robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.RETRACTING;
    }

    // can retract when going here if needed
    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return robot.getExtension().getExtensionMotorPosition() > Extension.MIN_SEARCH_AND_COLLECT_POSITION;
    }

    @Override
    public Extension.StateType getNextStateType() {
        return Extension.StateType.FINDING_BLOCK;
    }
}
