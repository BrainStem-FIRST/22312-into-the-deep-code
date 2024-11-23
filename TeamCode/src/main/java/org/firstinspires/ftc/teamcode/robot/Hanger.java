package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.hangingStates.HoldHang;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class Hanger extends Subsystem {

    // TODO: find down and up tick values and threshold
    // down refers to the position the hanging goes to after it is on the bar
    public final static int FULL_DOWN_TICK = 0, HANG_DOWN_TICK = 2076, UP_TICK = 33000, DESTINATION_THRESHOLD = 20;
    public final static double HANG_DOWN_POWER = 1;
    public final static int HANG_TICK_THRESHOLD = 5;

    // 1 represents transitioning to final hang from base position
    // -1 represents transitioning from final hang to base position
    private int movementDir = 0;

    public enum StateType {
        FULL_DOWN,
        UP,
        HANG_DOWN,
        TRANSITION
    }
    private final MotorTransitionState<StateType> transitionState;
    private final DcMotorEx hangMotor;

    private final StateManager<StateType> stateManager;
    public Hanger(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        hangMotor = hwMap.get(DcMotorEx.class, "HangMotor");
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stateManager = new StateManager<>(StateType.FULL_DOWN);
        stateManager.addState(StateType.FULL_DOWN, new NothingState<>(StateType.FULL_DOWN));
        stateManager.addState(StateType.UP, new NothingState<>(StateType.UP));
        stateManager.addState(StateType.HANG_DOWN, new HoldHang());

        // TODO: set power of hanging in states
        transitionState = new MotorTransitionState<>(StateType.TRANSITION, hangMotor, DESTINATION_THRESHOLD);
        transitionState.setEncoderBounds(FULL_DOWN_TICK, UP_TICK);
        stateManager.addState(StateType.TRANSITION, transitionState);


        stateManager.setupStates(getRobot(), stateManager);
    }

    public StateManager<StateType> getStateManager() { return stateManager; }

    public DcMotorEx getHangMotor() {
        return hangMotor;
    }
    public MotorTransitionState<StateType> getTransitionState() {
        return transitionState;
    }
    public void setMovementDir(int movementDir) {
        this.movementDir = movementDir;
    }

    public void setHangMotorPosition(int position) {
        Subsystem.setMotorPosition(hangMotor, position);
    }
    public void setHangMotorPower(double power) {
        Subsystem.setMotorPower(hangMotor, power);
    }

    @Override
    public void update(double dt) {
        stateManager.update(dt);

        // auto transitioning hanging to lift robot once motor has reached high enough height
        if(stateManager.getActiveStateType() == StateType.UP)
            if(movementDir == 1)
                transitionState.setGoalState(HANG_DOWN_TICK, StateType.HANG_DOWN);
            else if(movementDir == -1)
                transitionState.setGoalState(FULL_DOWN_TICK, StateType.FULL_DOWN);
    }
}
