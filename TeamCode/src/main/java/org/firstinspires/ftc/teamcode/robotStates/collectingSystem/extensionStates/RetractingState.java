package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class RetractingState extends RobotState<Extension.StateType> {

    public RetractingState() {
        super(Extension.StateType.RETRACTING);
    }

    @Override
    public void execute() {
        robot.getExtension().setExtensionMotorPower(Extension.RETRACT_POWER);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Extension.StateType.FINDING_BLOCK;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        // when the switch state returns false it means it has detected the extension
        return !robot.getExtension().getMagnetResetSwitch().getState();
    }

    @Override
    public Extension.StateType getNextStateType() {
        return Extension.StateType.IN;
    }
}
