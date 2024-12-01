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
    public final static double HANG_HOLD_POWER = -0.3;

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

    public void setHangMotorPosition(int position) {
        Subsystem.setMotorPosition(hangMotor, position);
    }
    public void setHangMotorPower(double power) {
        Subsystem.setMotorPower(hangMotor, power);
    }

    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }
}
