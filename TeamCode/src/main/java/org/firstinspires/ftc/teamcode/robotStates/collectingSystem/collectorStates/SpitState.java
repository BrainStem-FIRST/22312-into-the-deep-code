package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class SpitState extends RobotState<Collector.StateType> {

    public SpitState() {
        super(Collector.StateType.SPITTING);
    }

    @Override
    public void execute() {
        robot.getCollector().setSpindleMotorPower(-Collector.MAX_SPIN_POWER);
    }

    @Override
    public boolean canEnter() {
        return stateManager.getActiveStateType() == Collector.StateType.COLLECTING;
    }

    @Override
    public boolean canBeOverridden() {
        return true;
    }

    @Override
    public boolean isDone() {
        return time >= Collector.SPITTING_TIME;
    }

    @Override
    public Collector.StateType getNextStateType() {
        // state of collecting system will always be out when this function is called
        // bc only way it won't be is when extension is retracting, in retracting we override spitting and set state to hinge up
        // there should be no case where you want to keep spitting during retraction of extension, that would mean the opposing block would just act in way of your movement
        return Collector.StateType.COLLECTING;
    }
}
