package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.grabberStates;

import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class ClosingState extends RobotState<Grabber.StateType> {
    public ClosingState() {
        super(Grabber.StateType.CLOSING);
    }
    @Override
    public void execute() {
        robot.getGrabber().getGrabServo().setPosition(Grabber.CLOSE_POSITION);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Grabber.StateType.OPEN || stateManager.getActiveStateType() == Grabber.StateType.OPENING;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return time > 0.5;
        //return Math.abs(robot.getGrabber().getGrabServo().getPosition() - Grabber.CLOSE_POSITION) < Grabber.DESTINATION_THRESHOLD;
    }

    @Override
    public Grabber.StateType getNextStateType() {
        return Grabber.StateType.CLOSED;
    }
}
