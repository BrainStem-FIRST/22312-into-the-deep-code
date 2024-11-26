package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class InState extends RobotState<Extension.StateType> {
    public InState() {
        super(Extension.StateType.IN);
    }

    @Override
    public void execute() {
        if(robot.getExtension().hitRetractHardStop()) {
            robot.getExtension().setExtensionMotorPower(0);
            robot.getExtension().getExtensionMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else
            robot.getExtension().setExtensionMotorPower(Extension.RETRACT_POWER_IN);
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
