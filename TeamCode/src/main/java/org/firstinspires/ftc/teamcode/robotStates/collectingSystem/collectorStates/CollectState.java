package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class CollectState extends RobotState<Collector.StateType> {

    public static int BLOCK_COLOR_VALIDATION_FRAMES = 5;

    // how many frames the collector has had the block with the same color (to reduce noise)
    public int blockColorFrame;
    public CollectState() {

        super(Collector.StateType.COLLECTING);
        blockColorFrame = 0;
    }

    @Override
    public void execute() {
        robot.getCollector().setSpindleMotorPower(Collector.MAX_SPIN_POWER);

        if (robot.getCollector().getBlockColorSensor().getBlockColor() == BlockColor.NONE)
            blockColorFrame = 0;
        else
            blockColorFrame++;
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Collector.StateType.HINGE_DOWN ||
                stateManager.getActiveStateType() == Collector.StateType.SPITTING;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return robot.getCollector().getBlockColorSensor().getBlockColor() != BlockColor.NONE &&
                blockColorFrame >= BLOCK_COLOR_VALIDATION_FRAMES;
    }

    @Override
    public Collector.StateType getNextStateType() {
        if (robot.getCollector().hasValidBlockColor())
            return Collector.StateType.HINGE_UP;
        return Collector.StateType.SPITTING;
    }
}
