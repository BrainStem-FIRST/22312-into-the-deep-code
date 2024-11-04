package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class RetractingState extends RobotState<Extension.StateType> {

    public RetractingState() {
        super(Extension.StateType.RETRACTING);
    }

    @Override
    public void execute() {
        robot.getExtension().setExtensionMotorPosition(Extension.RETRACTED_POSITION);
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
        // TODO: incorporate the magnet sensors on back of lift (would say retracting is done when magnet activates AND delta position is less than threshold)
        return Math.abs(robot.getExtension().getExtensionMotor().getCurrentPosition() - Extension.RETRACTED_POSITION) < Extension.RETRACTED_THRESHOLD;
    }

    @Override
    public Extension.StateType getNextStateType() {
        return Extension.StateType.IN;
    }
}
