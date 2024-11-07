package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class CollectState extends RobotState<Collector.StateType> {

    public static int BLOCK_COLOR_VALIDATION_FRAMES = 5;

    // how many frames the collector has had the block with the same color (to reduce noise)
    private int blockColorFrame;
    private BlockColor prevBlockColor;
    public CollectState() {

        super(Collector.StateType.COLLECTING);
        blockColorFrame = 0;
        prevBlockColor = BlockColor.NONE;
    }

    @Override
    public void execute() {
        robot.getCollector().setSpindleMotorPower(Collector.MAX_SPIN_POWER);

        BlockColor curBlockColor = robot.getCollector().getBlockColorSensor().getBlockColor();
        if (curBlockColor == prevBlockColor && robot.getCollector().getBlockColorSensor().getBlockColor() != BlockColor.NONE) {
            blockColorFrame++;
            // adding block color to robot if done
            if(blockColorFrame > BLOCK_COLOR_VALIDATION_FRAMES)
                robot.setBlockColorHeld(curBlockColor);
        }
        else
            blockColorFrame = 0;
        prevBlockColor = curBlockColor;
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
