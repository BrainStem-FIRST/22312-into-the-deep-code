package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftSubsystem;

public class LiftingSystemTele {
    private final LiftTele lift;
    private final ArmTele arm;
    private final GrabberTele grabber;

    // describes transition to these states; not actual execution
    public enum State {
        COLLECT_TROUGH, BLOCK_DROP, SPECIMEN_PICKUP, SPECIMEN_RAM, BASKET_DROP
    }
    // states can be split into two parts: the transition to the state and the aftermath after the prep (could be at rest)
    private State state = State.COLLECT_TROUGH;
    private boolean curStateDone = false; // currently describes if state currently in is executing (ie not in transition)

    public LiftingSystemTele(HardwareMap hw, Telemetry telemetry, AllianceColor allianceColor) {
        lift = new LiftTele(hw, telemetry, allianceColor);
        arm = new ArmTele(hw, telemetry, allianceColor);
        grabber = new GrabberTele(hw, telemetry, allianceColor);
    }

    // should be called continuously
    public void update() {
        switch(state) {
            case COLLECT_TROUGH:
                pickupFromTrough();
                break;
            case BLOCK_DROP:
                prepareBlockDrop();
                break;
            case SPECIMEN_PICKUP:
                pickupSpecimen();
            case SPECIMEN_RAM:
                ramSpecimen();
                break;
            case BASKET_DROP:
                depositInBasket();
                break;
        }

        // updating actions of subsystems based on current lifting system state
        lift.update();
        arm.update();
        grabber.update();
    }


    // trough pick up functions
    public void pickupFromTrough() {
        if(lift.setGoalState(LiftSubsystem.State.TROUGH) && arm.setGoalState(LiftSubsystem.State.TROUGH) && grabber.setGoalState(LiftSubsystem.State.TROUGH)) {
            curStateDone = grabber.executeCurrentState();
        }
    }

    // human player pick up functions
    public void pickupSpecimen() {
        if(lift.setGoalState(LiftSubsystem.State.BACK_DROP) && arm.setGoalState(LiftSubsystem.State.BACK_DROP) && grabber.setGoalState(LiftSubsystem.State.BACK_DROP))
            curStateDone = grabber.executeCurrentState();
    }

    // specimen ram functions
    public void ramSpecimen() { // have two actions that need to be executed
        if(lift.setGoalState(LiftSubsystem.State.RAM) && arm.setGoalState(LiftSubsystem.State.RAM) && grabber.setGoalState(LiftSubsystem.State.RAM)) // if ready, then move lift
            if(lift.executeCurrentState())
                curStateDone = grabber.executeCurrentState();
    }


    // basket deposit functions
    public void depositInBasket() {
        if(lift.setGoalState(LiftSubsystem.State.BASKET) && arm.setGoalState(LiftSubsystem.State.BASKET) && grabber.setGoalState(LiftSubsystem.State.BASKET))
            curStateDone = grabber.executeCurrentState();
    }

    // dropping block for human player functions
    public void prepareBlockDrop() { // assumes you already have block in grabber
        if(lift.setGoalState(LiftSubsystem.State.BACK_DROP) && arm.setGoalState(LiftSubsystem.State.BACK_DROP) && grabber.setGoalState(LiftSubsystem.State.BACK_DROP))
            curStateDone = grabber.executeCurrentState();
    }

    // getters/setters
    public State getState() {
        return state;
    }
    public void setState(State state) {
        this.state = state;
    }
    public boolean getCurStateDone() {
        return curStateDone;
    }
}
