package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class RamToTroughState extends RobotState<LiftingSystem.StateType> {
    public RamToTroughState() {
        super(LiftingSystem.StateType.RAM_TO_TROUGH);
    }
    @Override
    public void execute() {
        if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED)
            robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
        // resetting lifting system once grabber lets go of specimen (which happens on user input)
        else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
            robot.getGrabber().setHasSpecimen(false);
            if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.SPECIMEN_HANG)
                robot.getArm().getTransitionState().setGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER);
            else if (robot.getArm().getTransitionState().getTime() >= Arm.SPECIMEN_HANG_TO_UP_TIME)
                robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
        }
    }

    @Override
    public boolean canEnter() {
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.SPECIMEN_RAM;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return LiftingSystem.StateType.TROUGH;
    }
}