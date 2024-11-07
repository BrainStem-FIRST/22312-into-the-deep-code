package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.grabberStates;

import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class OpeningState extends RobotState<Grabber.StateType> {
    public OpeningState() {
        super(Grabber.StateType.OPENING);
    }
    @Override
    public void execute() {
        robot.getGrabber().getGrabServo().setPosition(Grabber.OPEN_POSITION);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Grabber.StateType.CLOSED ||
                stateManager.getActiveStateType() == Grabber.StateType.CLOSING;

    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return time > 0.5;
        //return Math.abs(robot.getGrabber().getGrabServo().getPosition() - Grabber.OPEN_POSITION) < Grabber.DESTINATION_THRESHOLD;
    }

    @Override
    public Grabber.StateType getNextStateType() {
        return Grabber.StateType.OPEN;
    }
}
