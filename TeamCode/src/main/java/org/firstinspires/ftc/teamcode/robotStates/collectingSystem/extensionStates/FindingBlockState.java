package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class FindingBlockState extends RobotState<Extension.StateType> {

    public FindingBlockState() {
        super(Extension.StateType.FINDING_BLOCK);
    }

    @Override
    public void execute(double dt) {
        double power = 0;
        // set extension target power
            if (robot.getInput().getGamepadTracker1().isRightBumperPressed())
                power = Extension.SEARCH_POWER;
            else if (robot.getInput().getGamepadTracker1().isLeftBumperPressed())
                power = -Extension.SEARCH_POWER;

        // hard stop
        if (robot.getExtension().getExtensionMotorPosition() > Extension.MAX_POSITION)
            power = Math.min(0, power);
        if (robot.getExtension().getExtensionMotorPosition() < Extension.MIN_SEARCH_AND_COLLECT_POSITION)
            power = Math.max(0, power);

        // move extension
        robot.getExtension().setExtensionMotorPower(power);
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
}
