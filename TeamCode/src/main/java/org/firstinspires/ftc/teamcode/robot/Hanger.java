package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.hangingStates.HoldHang;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class Hanger extends Subsystem {

    // TODO: find down and up tick values and threshold
    // down refers to the position the hanging goes to after it is on the bar
    public final static int DOWN_TICK = 50, UP_TICK = 100;
    public final static double HANG_DOWN_POWER = 0.6;
    public final static int HANG_TICK_THRESHOLD = 5;

    public enum StateType {
        FULL_DOWN,
        GOING_UP,
        UP,
        GOING_DOWN,
        HANG_DOWN
    }
    private final DcMotorEx hangMotor;

    private final StateManager<StateType> stateManager;
    public Hanger(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        hangMotor = hwMap.get(DcMotorEx.class, "HangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stateManager = new StateManager<>(StateType.FULL_DOWN);
        stateManager.addState(StateType.FULL_DOWN, new NothingState<>(StateType.FULL_DOWN));
        stateManager.addState(StateType.UP, new NothingState<>(StateType.UP));
        stateManager.addState(StateType.HANG_DOWN, new HoldHang());

        MotorTransitionState<StateType> goingUpState = new MotorTransitionState<>(StateType.GOING_UP, hangMotor, HANG_TICK_THRESHOLD);
        stateManager.addState(StateType.GOING_UP, goingUpState);

        MotorTransitionState<StateType> goingDownState = new MotorTransitionState<>(StateType.GOING_DOWN, hangMotor, HANG_TICK_THRESHOLD);
        stateManager.addState(StateType.GOING_DOWN, goingDownState);

        stateManager.setupStates(getRobot(), stateManager);

        goingUpState.setGoalState(UP_TICK, StateType.UP);
        goingUpState.setGoalState(DOWN_TICK, StateType.HANG_DOWN);
    }

    public StateManager<StateType> getStateManager() { return stateManager; }

    public DcMotorEx getHangMotor() {
        return hangMotor;
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
