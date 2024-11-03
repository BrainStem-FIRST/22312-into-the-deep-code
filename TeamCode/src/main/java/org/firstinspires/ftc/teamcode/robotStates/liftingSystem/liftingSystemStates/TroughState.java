package org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftingSystemStates;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class TroughState extends RobotState<LiftingSystem.StateType> {
    public TroughState() {
        super(LiftingSystem.StateType.TROUGH);
    }
    @Override
    public void execute() {
        if(robot.getCollector().getStateManager().getActiveStateType() == Collector.StateType.NOTHING &&
                robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.IN &&
                robot.getCollector().hasValidBlockColor())
            robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.CLOSING);
    }

    @Override
    public boolean canEnter() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH &&
                robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DOWN &&
                robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN;
    }

    // only can be overridden if grabber is not closing in on block
    @Override
    public boolean canBeOverridden() {
        return robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN;
    }

    @Override
    public boolean isDone() {
        return robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        // if block is yellow, transition to basket, and if block is alliance color then transition to drop off to human player
        return robot.getCollector().getBlockColor() == Collector.BlockColor.YELLOW ? LiftingSystem.StateType.TROUGH_TO_BASKET : LiftingSystem.StateType.TROUGH_TO_DROP_AREA;
    }
}
