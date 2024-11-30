package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class SpitState extends RobotState<Collector.StateType> {

    // after the block color sensor stops detecting the block, still spit for 1 second
    public static double SAFETY_TIME = 0.8;
    private double startTime;
    public SpitState() {
        super(Collector.StateType.SPITTING);
        startTime = 0;
    }


    @Override
    public void execute() {

        // spit
        robot.getCollector().setSpindleMotorPower(-Collector.MAX_SPIN_POWER);

        // get the starting time to track safety
        if (robot.getCollector().getBlockColorSensor().getBlockColor() == BlockColor.NONE && startTime == 0)
            startTime = time;
        // reset the starting time
        if (robot.getCollector().getBlockColorSensor().getBlockColor() != BlockColor.NONE)
            startTime = 0;
    }

    @Override
    public boolean canEnter() {
        return true;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return robot.getCollector().getBlockColorSensor().getBlockColor() == BlockColor.NONE && time - startTime > SAFETY_TIME;
    }

    @Override
    public Collector.StateType getNextStateType() {
        return Collector.StateType.COLLECTING;
    }
}
