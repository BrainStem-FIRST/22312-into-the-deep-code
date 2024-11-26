package org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates;

import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class SpitTempState extends RobotState<Collector.StateType> {

    // keeps track of how many frames - not based on time - it has been running for
    private int framesRunning;

    public SpitTempState() {
        super(Collector.StateType.SPITTING_TEMP);
        framesRunning = 0;
    }

    // resets framesRunning - meant to be called continuously
    // as soon as you stop calling it, the state will stop
    public void continueRunning() {
        framesRunning = 0;
    }

    @Override
    public void execute() {
        // spit
        robot.getCollector().setSpindleMotorPower(-Collector.SPIT_TEMP_POWER);
        framesRunning++;

        // hinge to middle
        robot.getHinge().getTransitionState().setGoalState(Hinge.HINGE_MIDDLE_POSITION, Hinge.StateType.MIDDLE);
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
        return framesRunning > 1;
    }

    @Override
    public Collector.StateType getNextStateType() {
        return Collector.StateType.COLLECTING;
    }
}
