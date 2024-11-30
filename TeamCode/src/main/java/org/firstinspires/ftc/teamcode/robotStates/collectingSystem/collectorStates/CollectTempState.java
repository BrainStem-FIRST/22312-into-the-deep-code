package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class CollectTempState extends RobotState<Collector.StateType> {

    // keeps track of how many frames - not based on time - it has been running for
    private int framesRunning;

    public CollectTempState() {
        super(Collector.StateType.COLLECTING_TEMP);
        framesRunning = 0;
    }

    // resets framesRunning - meant to be called continuously
    // as soon as you stop calling it, the state will stop
    public void continueRunning() {
        framesRunning = 0;
    }

    @Override
    public void execute() {
        // collect
        robot.getCollector().setSpindleMotorPower(Collector.COLLECT_TEMP_POWER);
        framesRunning++;

        // tell robot that block is not ready for transfer
        robot.setBlockReadyForTransfer(false);
    }

    @Override
    public boolean canEnter() {
        return robot.canCollect();
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return framesRunning > 1;
    }

    @Override
    public Collector.StateType getNextStateType() {
        return Collector.StateType.NOTHING;
    }
}
