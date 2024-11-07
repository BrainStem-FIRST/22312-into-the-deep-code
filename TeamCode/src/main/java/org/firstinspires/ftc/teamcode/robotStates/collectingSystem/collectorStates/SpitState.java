package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class SpitState extends RobotState<Collector.StateType> {

    // after the block color sensor stops detecting the block, still spit for 1 second
    public static double SAFETY_TIME = 1;
    private double startTime;
    public SpitState() {
        super(Collector.StateType.SPITTING);
        startTime = 0;
    }

    @Override
    public void execute() {
        robot.getCollector().setSpindleMotorPower(-Collector.MAX_SPIN_POWER);

        // get the starting time
        if (robot.getCollector().getBlockColorSensor().getBlockColor() == BlockColor.NONE && startTime == 0)
            startTime = time;
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Collector.StateType.COLLECTING;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return time - startTime >= SAFETY_TIME;
    }

    @Override
    public Collector.StateType getNextStateType() {
        if(robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.RETRACTING)
            return Collector.StateType.HINGE_UP;
        return Collector.StateType.COLLECTING;
    }
}
