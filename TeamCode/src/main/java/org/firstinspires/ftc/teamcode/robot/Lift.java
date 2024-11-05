package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

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
        TROUGH, TROUGH_SAFETY, DROP_OFF, RAM_BEFORE, RAM_AFTER, BASKET_SAFETY, BASKET, TRANSITION
    }
    private final StateManager<StateType> stateManager;
    private final MotorTransitionState<StateType> transitionState;

    public Lift(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stateManager = new StateManager<>(StateType.TROUGH);

        NothingState<StateType> troughState = new NothingState<>(StateType.TROUGH);
        troughState.addMotor(liftMotor);
        stateManager.addState(StateType.TROUGH, troughState);

        NothingState<StateType> troughSafetyState = new NothingState<>(StateType.TROUGH_SAFETY);
        troughSafetyState.addMotor(liftMotor);
        stateManager.addState(StateType.TROUGH_SAFETY, troughSafetyState);

        NothingState<StateType> dropOffState = new NothingState<>(StateType.DROP_OFF);
        dropOffState.addMotor(liftMotor);
        stateManager.addState(StateType.DROP_OFF, dropOffState);

        NothingState<StateType> ramBeforeState = new NothingState<>(StateType.RAM_BEFORE);
        ramBeforeState.addMotor(liftMotor);
        stateManager.addState(StateType.RAM_BEFORE, ramBeforeState);

        NothingState<StateType> ramAfterState = new NothingState<>(StateType.RAM_AFTER);
        ramAfterState.addMotor(liftMotor);
        stateManager.addState(StateType.RAM_AFTER, ramAfterState);

        NothingState<StateType> basketSafetyState = new NothingState<>(StateType.BASKET_SAFETY);
        basketSafetyState.addMotor(liftMotor);
        stateManager.addState(StateType.BASKET_SAFETY, basketSafetyState);

        NothingState<StateType> basketState = new NothingState<>(StateType.BASKET);
        basketState.addMotor(liftMotor);
        stateManager.addState(StateType.BASKET, basketState);

        this.transitionState = new MotorTransitionState<>(StateType.TRANSITION, liftMotor, DESTINATION_THRESHOLD);
        stateManager.addState(StateType.TRANSITION, this.transitionState);

        stateManager.setupStates(robot, stateManager);
        stateManager.tryEnterState(StateType.TROUGH);
    }

    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }

    public DcMotorEx getLiftMotor() {
        return liftMotor;
    }
    public StateManager<StateType> getStateManager() {
        return stateManager;
    }
    public MotorTransitionState<StateType> getTransitionState() {
        return transitionState;
    }
}

