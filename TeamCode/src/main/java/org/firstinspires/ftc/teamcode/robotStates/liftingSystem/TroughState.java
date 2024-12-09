package org.firstinspires.ftc.teamcode.robotStates.liftingSystem;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
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
        // handling actual transfer
        if(robot.shouldTransfer()) {
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TRANSFER);
        }
        // checking if need to transfer again and handling transitions after transfer and checking if suddenly cannot transfer
        else {
            // this would handle automatic transition to transfer; note that it depends on color sensor
            if(robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.IN
            && robot.getCollector().hasValidBlockColor()) {
                robot.setShouldTransfer(true); // next frame transfer will occur
            }
            // if have successful transfer and is depositing
            else if (robot.getGrabber().hasBlock() && robot.isDepositing())
                robot.getArm().getTransitionState().setGoalState(Arm.BASKET_SAFETY_POS, Arm.StateType.BASKET_SAFETY, Arm.TRANSFER_TO_BASKET_SAFETY_TIME);
            // prepping for specimen pickup at human player station
            else if (!robot.getLiftingSystem().getStayInTrough())
                robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_DROP_AREA);
        }
    }

    @Override
    public boolean canEnter() {
        // lift must properly be lowered and set before entering
        return robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.DROP_AREA_TO_TROUGH
                || robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.RAM_TO_TROUGH;
    }

    // only can be overridden if lift is at trough safety and if grabber has block
    @Override
    public boolean canBeOverridden() {
        return robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY;
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public LiftingSystem.StateType getNextStateType() {
        return null;
    }
}
