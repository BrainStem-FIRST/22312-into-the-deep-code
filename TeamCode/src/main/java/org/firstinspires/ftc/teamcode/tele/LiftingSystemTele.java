package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftSubsystem;

public class LiftingSystemTele<T> {
    private final LiftTele lift;
    private final ArmTele arm;
    private final GrabberTele grabber;

    // describes transition to these states; not actual execution
    public enum State {
        TROUGH, BLOCK_DROP, SPECIMEN_PICKUP, SPECIMEN_RAM, BASKET_DROP
    }
    // states can be split into two parts: the transition to the state and the aftermath after the prep (could be at rest)
    private State state = State.TROUGH;
    private boolean curStateReady = false; // true when prep for state is done
    private boolean curStateDone = false; // true when state execution is done

    public LiftingSystemTele(HardwareMap hw, Telemetry telemetry, AllianceColor allianceColor) {
        lift = new LiftTele(hw, telemetry, allianceColor);
        arm = new ArmTele(hw, telemetry, allianceColor);
        grabber = new GrabberTele(hw, telemetry, allianceColor);
    }

    public LiftSubsystem.State convertState(State state) {
        switch(state) {
            case TROUGH:
                return LiftingSystem.State.TROUGH;
            case BLOCK_DROP:
                return LiftingSystem.State.BACK_DROP;
            case SPECIMEN_PICKUP:
                return LiftingSystem.State.BACK_DROP;
            case SPECIMEN_RAM:
                return LiftingSystem.State.RAM;
            case BASKET_DROP:
                return LiftinSystem.State.BASKET;
        }
    }

    // should be called continuously
    public void update() {
        LiftSubsystem.State s = convertState(state);

        lift.setGoalState(s);
        arm.setGoalState(s);
        grabber.setGoalState(s);

        curStateReady = lift.getGoalState() == null && arm.getGoalState() == null && grabber.getGoalState() == null;

        // updating actions of subsystems based on current lifting system state
        lift.update();
        arm.update();
        grabber.update();
    }

    // getters/setters
    public State getState() {
        return state;
    }
    public void setState(State state) {
        this.state = state;
        curStateReady = false;
        curStateDone = false;
    }
    public boolean getCurStateReady() {
        return curStateReady;
    }
    public boolean getCurStateDone() {
        return curStateDone;
    }
}
