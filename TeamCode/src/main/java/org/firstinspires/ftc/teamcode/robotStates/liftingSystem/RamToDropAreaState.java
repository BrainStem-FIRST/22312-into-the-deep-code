package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class RamToDropAreaState extends RobotState<LiftingSystem.StateType> {
    public RamToDropAreaState() {
        super(LiftingSystem.StateType.RAM_TO_DROP_AREA);
    }
    @Override
    public void execute() {
        // actually ramming using lift
        if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_BEFORE)
            robot.getLift().getTransitionState().setGoalState(robot.getLift().getRamAfterPos(), Lift.StateType.RAM_AFTER);

        // resetting after ram
        else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_AFTER) {
            // releasing specimen
            if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                robot.getGrabber().setHasSpecimen(false);
                robot.setIsDepositing(true); // resets depositing mode to true
            }

            // resetting lifting system once grabber lets go of specimen
            else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                if(robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.SPECIMEN_RAM)
                    robot.getArm().getTransitionState().setGoalState(Arm.DROP_OFF_POS, Arm.StateType.DROP_OFF, Arm.DROP_OFF_TO_SPECIMEN_RAM_TIME);

                // resetting lift to drop off position once arm clears ramming bar or finishes transition (in case the transition time doesn't work)
                else if(robot.getArm().getTransitionState().getTime() >= Arm.SPECIMEN_RAM_TO_UP_TIME
                || robot.getArm().getStateManager().getActiveStateType() != Arm.StateType.TRANSITION)
                    if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_AFTER)
                        robot.getLift().getTransitionState().setGoalState(Lift.DROP_AREA_POS, Lift.StateType.DROP_AREA);
                    // adding kI to lift pid once below/at trough safety
                    else if(robot.getLift().getLiftMotor().getCurrentPosition() <= Lift.TROUGH_SAFETY_POS)
                        robot.getLift().getTransitionState().getPid().setTempKI(Lift.TRANSFER_KI);
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
        return (robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.DROP_OFF && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.DROP_AREA);
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        //return robot.getLiftingSystem().subsystemsAtTrough() ? LiftingSystem.StateType.TROUGH : LiftingSystem.StateType.DROP_AREA;
        return LiftingSystem.StateType.DROP_AREA;
    }
}