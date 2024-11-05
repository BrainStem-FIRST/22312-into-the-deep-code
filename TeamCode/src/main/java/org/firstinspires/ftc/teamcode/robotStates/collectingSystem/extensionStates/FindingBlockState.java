package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class FindingBlockState extends RobotState<Extension.StateType> {

    public FindingBlockState() {
        super(Extension.StateType.FINDING_BLOCK);
    }

    @Override
    public void execute() {
        // move extension
        if (robot.getExtension().getRunMode() == DcMotor.RunMode.RUN_TO_POSITION)
            robot.getExtension().setExtensionMotorPosition(robot.getExtension().getTargetPosition());
        else if (robot.getExtension().getRunMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            robot.getExtension().setExtensionMotorPower(robot.getExtension().getTargetPower());

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
