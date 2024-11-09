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
    public static final int MAX_POS = 3050,
        TROUGH_POS = 0,
        TROUGH_SAFETY_POS = 350, // position where arm can safely raise without colliding with collector
        DROP_AREA_POS = 0,

        LOW_RAM_BEFORE_POS = 320,
        LOW_RAM_AFTER_POS = 565,
        HIGH_RAM_BEFORE_POS = 1500,
        HIGH_RAM_AFTER_POS = 1700,

        LOW_BASKET_POS = 1700,
        HIGH_BASKET_POS = 3040;

    public static final int DESTINATION_THRESHOLD = 30;
    public enum StateType {
        TROUGH, TROUGH_SAFETY, DROP_AREA, RAM_BEFORE, RAM_AFTER, BASKET_DEPOSIT, BASKET_SAFETY, TRANSITION
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

        NothingState<StateType> dropAreaState = new NothingState<>(StateType.DROP_AREA);
        dropAreaState.addMotor(liftMotor);
        stateManager.addState(StateType.DROP_AREA, dropAreaState);

        NothingState<StateType> ramBeforeState = new NothingState<>(StateType.RAM_BEFORE);
        ramBeforeState.addMotor(liftMotor);
        stateManager.addState(StateType.RAM_BEFORE, ramBeforeState);

        NothingState<StateType> ramAfterState = new NothingState<>(StateType.RAM_AFTER);
        ramAfterState.addMotor(liftMotor);
        stateManager.addState(StateType.RAM_AFTER, ramAfterState);

        NothingState<StateType> basketState = new NothingState<>(StateType.BASKET_DEPOSIT);
        basketState.addMotor(liftMotor);
        stateManager.addState(StateType.BASKET_DEPOSIT, basketState);

        NothingState<StateType> basketSafetyState = new NothingState<>(StateType.BASKET_SAFETY);
        basketSafetyState.addMotor(liftMotor);
        stateManager.addState(StateType.BASKET_SAFETY, basketSafetyState);

        this.transitionState = new MotorTransitionState<>(StateType.TRANSITION, liftMotor, DESTINATION_THRESHOLD);
        stateManager.addState(StateType.TRANSITION, this.transitionState);

        stateManager.setupStates(robot, stateManager);
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
    public int getRamBeforePos() {
        return getRobot().isHighRam() ? HIGH_RAM_BEFORE_POS : LOW_RAM_BEFORE_POS;
    }
    public int getRamAfterPos() {
        return getRobot().isHighRam() ? HIGH_RAM_AFTER_POS : LOW_RAM_AFTER_POS;
    }
    public boolean atHighBasket() {
        return stateManager.getActiveStateType() == StateType.BASKET_DEPOSIT &&
                getTransitionState().getGoalStatePosition() == HIGH_BASKET_POS;
    }
    public boolean atLowBasket() {
        return stateManager.getActiveStateType() == StateType.BASKET_DEPOSIT &&
                getTransitionState().getGoalStatePosition() == LOW_BASKET_POS;
    }
}