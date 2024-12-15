package org.firstinspires.ftc.teamcode.robotStates.robot;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Hanger;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class PlayingState extends RobotState<BrainSTEMRobot.StateType> {

    public PlayingState() {
        super(BrainSTEMRobot.StateType.PLAYING);
    }

    @Override
    public void execute(double dt) {

        // drive train
        robot.getDriveTrain().updatePoseEstimate();

        // collecting system
        robot.getCollectingSystem().update(dt);
        robot.getCollector().update(dt);
        robot.getHinge().update(dt);
        robot.getExtension().update(dt);

        // lifting system
        robot.getLiftingSystem().update(dt);
        robot.getGrabber().update(dt);
        robot.getArm().update(dt);
        robot.getLift().update(dt);

        // hanging system
        robot.getHanger().update(dt);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == BrainSTEMRobot.StateType.SETTING_UP;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public BrainSTEMRobot.StateType getNextStateType() {
        return null;
    }
}
