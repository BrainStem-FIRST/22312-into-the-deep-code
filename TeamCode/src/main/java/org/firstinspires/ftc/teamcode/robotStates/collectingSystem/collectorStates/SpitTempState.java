package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class SpitTempState extends RobotState<Collector.StateType> {

    public SpitTempState() {
        super(Collector.StateType.SPITTING_TEMP);
    }

    @Override
    public void execute(double dt) {
        // spit
        robot.getCollector().setSpindleMotorPower(Collector.SPIT_TEMP_POWER);
        framesRunning++;

        // tell robot that block is not ready for transfer
        if (robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.IN)
            robot.setCanTransfer(false);
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
        return framesRunning > 1;
    }

    @Override
    public Collector.StateType getNextStateType() {
        return Collector.StateType.NOTHING;
    }
}
