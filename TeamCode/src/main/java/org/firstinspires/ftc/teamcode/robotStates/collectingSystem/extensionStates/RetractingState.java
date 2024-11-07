package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class RetractingState extends RobotState<Extension.StateType> {

    private int startPos;
    private double speedPercent;

    public RetractingState() {
        super(Extension.StateType.RETRACTING);
        speedPercent = 1;
        startPos = 0;
    }

    @Override
    public void execute() {
        if(isFirstTime())
            startPos = robot.getExtension().getExtensionMotor().getCurrentPosition();
        speedPercent *= robot.getExtension().getExtensionMotor().getCurrentPosition();
        robot.getExtension().setExtensionMotorPower(Extension.RETRACT_POWER * speedPercent);
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
        return robot.getExtension().hitRetractHardStop();
    }

    @Override
    public Extension.StateType getNextStateType() {
        return Extension.StateType.IN;
    }
}
