package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class FindingBlock extends RobotState<Extension.StateType> {

    public static final double SPEED_MODIFIER = 0.2;
    public FindingBlock() {
        super(Extension.StateType.FINDING_BLOCK);
    }

    @Override
    public void execute() {
        robot.getExtension().setExtensionMotorPower(gamepad2.left_stick_y * 0.2);

        // hard stop
        if (robot.getExtension().getExtensionMotor().getCurrentPosition() >= Extension.MAX_POSITION)
            robot.getExtension().setExtensionMotorPosition(0);
    }

    @Override
    public boolean canEnter() {
        return robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.IN;
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
        return null;
    }
}
