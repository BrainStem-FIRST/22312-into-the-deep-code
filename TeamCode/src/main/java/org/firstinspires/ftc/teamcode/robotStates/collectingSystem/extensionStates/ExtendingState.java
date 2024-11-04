package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class ExtendingState extends RobotState<Extension.StateType> {

    public ExtendingState() {
        super(Extension.StateType.EXTENDING);
    }
    @Override
    public void execute() {
        // TODO: would want to use PID here instead of setting target position
        robot.getExtension().setExtensionMotorPosition(Extension.EXTENDED_POSITION);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Extension.StateType.IN;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return Math.abs(robot.getExtension().getExtensionMotor().getCurrentPosition() - Extension.EXTENDED_POSITION) < Extension.THRESHOLD;
    }

    @Override
    public Extension.StateType getNextStateType() {
        return Extension.StateType.OUT;
    }
}
