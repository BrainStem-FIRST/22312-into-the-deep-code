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
    public void execute(double dt) {
        // actually ramming using lift
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_BEFORE)
            robot.getLift().getTransitionState().setGoalState(robot.getLift().getRamAfterPos(), Lift.StateType.RAM_AFTER);

        // resetting after ram
        else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_AFTER) {
            if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                robot.getGrabber().setHasSpecimen(false);
                robot.setIsDepositing(true); // resets depositing mode to true
                robot.getLiftingSystem().setStayInTrough(true);
            }
            // resetting lifting system once grabber lets go of specimen (which happens on user input)
            else if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.SPECIMEN_HANG)
                    robot.getArm().getTransitionState().setGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER, Arm.SPECIMEN_HANG_TO_UP_TIME + Arm.UP_TO_TRANSFER_TIME);
                else if (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER
                || robot.getArm().getTransitionState().getTime() >= Arm.SPECIMEN_HANG_TO_UP_TIME) {
                    robot.getLift().getTransitionState().setGoalState(Lift.TROUGH_SAFETY_POS, Lift.StateType.TROUGH_SAFETY);
                    robot.getArm().getTransitionState().setGoalState(Arm.TRANSFER_POS, Arm.StateType.TRANSFER, Arm.UP_TO_TRANSFER_TIME);
                }
            }
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