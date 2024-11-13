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
        int curPos = robot.getExtension().getExtensionMotor().getCurrentPosition();
        //if(isFirstTime()) {
            //robot.getExtension().getPid().reset();
            //robot.getExtension().getPid().setTarget(Extension.MIN_POSITION);
        //}
        if(robot.getInPidMode())
            robot.getExtension().setExtensionMotorPower(-robot.getExtension().getPid().update(curPos));
        else
            Subsystem.setMotorPosition(robot.getExtension().getExtensionMotor(), Extension.MIN_POSITION);
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
