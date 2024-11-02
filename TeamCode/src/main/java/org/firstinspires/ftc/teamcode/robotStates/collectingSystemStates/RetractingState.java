package org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.tele.CollectingSystemTele;

public class RetractingState extends CollectSystemState {

    public RetractingState() {
        super(CollectingSystemTele.StateType.RETRACTING);
    }

    @Override
    public void execute() {
        robot.getExtension().setExtensionMotorPosition(Extension.RETRACTED_POSITION);
        robot.getCollector().setHingeServoPosition(Collector.HINGE_UP_POSITION);

        // stop spindle motors once hinging is complete
        // run them at a slower speed during hinging motion (to keep block in place)
        if (Math.abs(robot.getCollector().getHingeServo().getPosition() - Collector.HINGE_UP_POSITION) < Collector.HINGE_THRESHOLD)
            robot.getCollector().setSpindleMotorPower(0);
        else
            robot.getCollector().setSpindleMotorPower(Collector.HOLD_SPIN_POWER);
    }

    @Override
    public boolean canEnter() {
        return collectingSystemTele.getStateManager().getActiveStateType() != CollectingSystemTele.StateType.IN &&
                collectingSystemTele.getStateManager().getActiveStateType() != CollectingSystemTele.StateType.SPITTING;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return Math.abs(robot.getExtension().getExtensionMotor().getCurrentPosition() - Extension.RETRACTED_POSITION) < Extension.THRESHOLD;
    }

    @Override
    public CollectingSystemTele.StateType getNextStateType() {
        return CollectingSystemTele.StateType.IN;
    }
}
