package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class InState extends RobotState<Extension.StateType> {
    public InState() {
        super(Extension.StateType.IN);
    }

    @Override
    public void execute(double dt) {
        if(!robot.getExtension().hitRetractHardStop())
            robot.getExtension().setTargetPower(Extension.RETRACT_POWER_IN);
        else if (robot.getLift().getTransitionState().getNextStateType() == Lift.StateType.TROUGH)
            robot.getExtension().setTargetPower(Extension.TRANSFER_POWER_IN);
        else {
            robot.getExtension().setTargetPower(0);
            robot.getExtension().getExtensionMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        robot.getExtension().setExtensionMotorPower(robot.getExtension().getTargetPower());

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
