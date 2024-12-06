package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.extensionStates;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class JumpToMin extends RobotState<Extension.StateType> {

    public static int THRESHOLD = 2;

    public JumpToMin() {
        super(Extension.StateType.JUMP_TO_MIN);
    }

    @Override
    public void execute() {

        // reset canTransfer so the actions of collecting and spitting temp of prev block do not affect
        // this collection cycle
        if (isFirstTime())
            robot.setCanTransfer(true);

        // go to min position
        robot.getExtension().setExtensionMotorPosition(Extension.MIN_SEARCH_AND_COLLECT_POSITION + 20);
    }

    @Override
    public boolean canEnter() {
        return robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.IN;
    }

    // can retract when going here if needed
    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return Subsystem.inRange(robot.getExtension().getExtensionMotor(), Extension.MIN_SEARCH_AND_COLLECT_POSITION, THRESHOLD)
                || robot.getExtension().getExtensionMotor().getCurrentPosition() > Extension.MIN_SEARCH_AND_COLLECT_POSITION;
    }

    @Override
    public Extension.StateType getNextStateType() {
        return Extension.StateType.FINDING_BLOCK;
    }
}
