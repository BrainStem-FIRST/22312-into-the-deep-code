package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class RetractingState extends RobotState<Extension.StateType> {

    public RetractingState() {
        super(Extension.StateType.RETRACTING);
    }

    @Override
    public void execute() {
        robot.telemetry.addData("in retracting state of extension", "");

        if(isFirstTime())
            robot.getExtension().getPid().reset();
        if(!robot.getInPidMode())
            Subsystem.setMotorPosition(robot.getExtension().getExtensionMotor(), Extension.MIN_POSITION);
        else
            Subsystem.setMotorPower(robot.getExtension().getExtensionMotor(),
                    robot.getExtension().getPid().update(robot.getExtension().getExtensionMotor().getCurrentPosition()));
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
