package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftSubsystem;

public class LiftingSystemTele {
    // TODO (INFRASTRUCTURE)
    // define delays in updateSubsystemStates function
    // finish writing execution functions in LiftSubsystem classes
    //
    public static final int LIFT_ARM_RESTRICTION = 10; // if lift is lower than 3 inches, arm has to be perfectly vertical
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
    private boolean curStateExecuting = false;
    private boolean curStateDone = false; // true when state execution is done

    public LiftingSystemTele(HardwareMap hw, Telemetry telemetry, AllianceColor allianceColor) {
        lift = new LiftTele(hw, telemetry, allianceColor);
        arm = new ArmTele(hw, telemetry, allianceColor);
        grabber = new GrabberTele(hw, telemetry, allianceColor);
    }

    public void updateSubsystemStates(State state) {
        switch(state) {
            case TROUGH:
                lift.setGoalState(Lift.State.TROUGH);
                arm.setGoalState(Arm.State.TROUGH);
                grabber.setGoalState(Grabber.State.TROUGH);
                break;
            case BLOCK_DROP:

                break;
            case SPECIMEN_PICKUP:
                break;
            case SPECIMEN_RAM:
                break;
            case BASKET_DROP:
                break;
            default:
                break;
        }
    }

    // should be called continuously
    public void update() {

        updateSubsystemStates(state);

        curStateReady = lift.getGoalState() == null && arm.getGoalState() == null && grabber.getGoalState() == null;

        // updating actions of subsystems based on current lifting system state
        lift.update();
        arm.update();
        grabber.update();
    }
    public void execCurrentState() {
        lift.executeCurrentState();
        arm.executeCurrentState();
        grabber.executeCurrentState();
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
    public boolean getCurStateExecuting() {
        return curStateExecuting;
    }
    public void setCurStateExecuting(boolean curStateExecuting) {
        this.curStateExecuting = curStateExecuting;
    }
    public LiftTele getLift() {
        return lift;
    }
    public ArmTele getArm() {
        return arm;
    }
    public GrabberTele getGrabber() {
        return grabber;
    }
}
