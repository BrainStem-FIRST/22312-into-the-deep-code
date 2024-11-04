package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class Lift extends Subsystem {
    private final DcMotorEx liftMotor;
    public static final int TROUGH_POS = 5,
        DROP_AREA_POS = 10,
        RAM_BEFORE_POS = 15,
        RAM_AFTER_POS = 20,
        BASKET_POS = 25,
        TROUGH_SAFETY_POS = 7, // position where arm can safely raise without colliding with collector
        BASKET_SAFETY_POS = 23; // position where arm can safely lower without colliding with basket
    public static final int DESTINATION_THRESHOLD = 5;
    public enum StateType {
        TROUGH, TROUGH_SAFETY, DROP_OFF, RAM_BEFORE, RAM_AFTER, BASKET, BASKET_SAFETY, TRANSITION
    }
    private final StateManager<StateType> stateManager;
    private final MotorTransitionState<StateType> transitionState;

    public Lift(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot, Gamepad gamepad1, Gamepad gamepad2) {
        super(hwMap, telemetry, allianceColor, robot, gamepad1, gamepad2);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stateManager = new StateManager<>(StateType.TROUGH);

        stateManager.addState(StateType.TROUGH, new NothingState<>(StateType.TROUGH));
        stateManager.addState(StateType.TROUGH_SAFETY, new NothingState<>(StateType.TROUGH_SAFETY));
        stateManager.addState(StateType.DROP_OFF, new NothingState<>(StateType.DROP_OFF));
        stateManager.addState(StateType.RAM_BEFORE, new NothingState<>(StateType.RAM_BEFORE));
        stateManager.addState(StateType.RAM_AFTER, new NothingState<>(StateType.RAM_AFTER));
        stateManager.addState(StateType.BASKET, new NothingState<>(StateType.BASKET));
        stateManager.addState(StateType.BASKET_SAFETY, new NothingState<>(StateType.BASKET_SAFETY));

        transitionState = new MotorTransitionState<>(StateType.TRANSITION, liftMotor, DESTINATION_THRESHOLD);
        stateManager.addState(StateType.TRANSITION, transitionState);

        stateManager.setupStates(robot, gamepad1, gamepad2, stateManager);
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
    public MotorTransitionState<StateType> getTransitionState() {
        return transitionState;
    }
}

