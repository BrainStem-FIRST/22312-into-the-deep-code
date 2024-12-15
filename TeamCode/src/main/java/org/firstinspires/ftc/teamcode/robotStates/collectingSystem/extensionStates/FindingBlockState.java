package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class FindingBlockState extends RobotState<Extension.StateType> {

    public FindingBlockState() {
        super(Extension.StateType.FINDING_BLOCK);
    }

    @Override
    public void executeOnEntered() {
        robot.setCanTransfer(false); // to override any current transfer
    }
    @Override
    public void execute(double dt) {
        // hard stop
        if (robot.getExtension().getExtensionMotor().getCurrentPosition() > Extension.MAX_POSITION)
            robot.getExtension().setTargetPower(Math.min(0, robot.getExtension().getTargetPower()));
        if (robot.getExtension().getExtensionMotor().getCurrentPosition() < Extension.MIN_SEARCH_AND_COLLECT_POSITION)
            robot.getExtension().setTargetPower(Math.max(0, robot.getExtension().getTargetPower()));

        // move extension
        robot.getExtension().setExtensionMotorPower(robot.getExtension().getTargetPower());
    }

    @Override
    public boolean canEnter() {
        return robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.JUMP_TO_MIN
                || robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.RETRACTING;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    // waiting for other states to override
    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public Extension.StateType getNextStateType() {
        return Extension.StateType.FINDING_BLOCK;
    }
}
