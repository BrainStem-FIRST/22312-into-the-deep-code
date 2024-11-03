package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.liftStates.TransitionState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class Lift extends Subsystem {
    private final DcMotorEx liftMotor;
    public static final int TROUGH_POSITION = 5,
        DROP_AREA_POSITION = 10,
        RAM_BEFORE_POSITION = 15,
        RAM_AFTER_POSITION = 20,
        BASKET_POSITION = 25,
        TROUGH_SAFETY_POSITION = 7, // position where arm can safely raise without colliding with collector
        BASKET_SAFETY_POSITION = 23; // position where arm can safely lower without colliding with basket
    public static final int DESTINATION_THRESHOLD = 5;
    public enum StateType {
        TROUGH, DROP_OFF, RAM_BEFORE, RAM_AFTER, BASKET, TRANSITION
    }
    private final StateManager<StateType> stateManager;

    public Lift(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot, Gamepad gamepad) {
        super(hwMap, telemetry, allianceColor, robot, gamepad);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stateManager = new StateManager<>(StateType.TROUGH);

        stateManager.addState(StateType.TROUGH, new NothingState<>(StateType.TROUGH));
        stateManager.addState(StateType.DROP_OFF, new NothingState<>(StateType.DROP_OFF));
        stateManager.addState(StateType.RAM_BEFORE, new NothingState<>(StateType.RAM_BEFORE));
        stateManager.addState(StateType.RAM_AFTER, new NothingState<>(StateType.RAM_AFTER));
        stateManager.addState(StateType.BASKET, new NothingState<>(StateType.BASKET));
        stateManager.addState(StateType.TRANSITION, new TransitionState());

        stateManager.setupStates(robot, gamepad, stateManager);
        stateManager.tryEnterState(StateType.TROUGH);
    }

    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }

    public DcMotorEx getLiftMotor() {
        return liftMotor;
    }
    public void setLiftPosition(int position) {
        setMotorPosition(liftMotor, position);
    }
    public StateManager<StateType> getStateManager() {
        return stateManager;
    }
}

