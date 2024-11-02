package org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.tele.CollectingSystemTele;
import org.firstinspires.ftc.teamcode.robotStates.RobotStateTele;

public class ExtendingState extends CollectSystemState {

    public ExtendingState() {
        super(CollectingSystemTele.StateType.EXTENDING);
    }
    @Override
    public void execute() {
        robot.getExtension().setExtensionMotorPosition(Extension.EXTENDED_POSITION);
        robot.getCollector().setHingeServoPosition(Collector.HINGE_DOWN_POSITION);
    }

    @Override
    public boolean canEnter() {
        return collectingSystemTele.getStateManager().getActiveStateType() == CollectingSystemTele.StateType.IN ||
                collectingSystemTele.getStateManager().getActiveStateType() == CollectingSystemTele.StateType.RETRACTING;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return Math.abs(robot.getExtension().getExtensionMotor().getCurrentPosition() - Extension.EXTENDED_POSITION) < Extension.THRESHOLD;
    }

    @Override
    public CollectingSystemTele.StateType getNextStateType() {
        return CollectingSystemTele.StateType.COLLECTING;
    }
}
