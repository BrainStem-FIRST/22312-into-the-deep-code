package org.firstinspires.ftc.teamcode.robotStates.robot;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Hanger;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class PlayingState extends RobotState<BrainSTEMRobot.StateType> {

    public PlayingState() {
        super(BrainSTEMRobot.StateType.PLAYING);
    }

    //  NOTE: COLLECTING SYSTEM NEEDS TO BE UPDATED BEFORE LIFTING SYSTEM TO ENSURE COLOR SENSOR VALUES ARE UP TO DATE WHEN LIFTING SYSTEM USES THEM
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

        if (isFirstTime())
            robot.getHanger().getTransitionState().setGoalState(Hanger.UP_TICK, Hanger.StateType.UP);
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
