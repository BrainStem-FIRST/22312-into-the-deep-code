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
        double power = 0;
        if(!robot.getExtension().hitRetractHardStop())
            power = Extension.RETRACT_POWER_IN;
        else if (robot.getLift().getTransitionState().getNextStateType() == Lift.StateType.TROUGH)
            power = Extension.TRANSFER_POWER_IN;

        robot.getExtension().setExtensionMotorPower(power);

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
