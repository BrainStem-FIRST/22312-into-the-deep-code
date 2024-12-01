package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.TimedAction;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class Lift extends Subsystem {
    public final static double FULL_POWER = 0.9;
    private final DcMotorEx liftMotor;
    private final PIDController pid;
    // TODO: find low and high basket safety positions for lift (determines when arm can start rotating into basket deposit position)
    public static final int TROUGH_POS = -5,
        TROUGH_SAFETY_POS = 350, // position where arm can safely raise without colliding with collector
        DROP_AREA_POS = 5,
        DROP_AREA_AFTER_POS = 30,
        LOW_RAM_BEFORE_POS = 320,
        LOW_RAM_AFTER_POS = 595,
        HIGH_RAM_BEFORE_POS = 750,
        HIGH_RAM_AFTER_POS = 1860,

        LOW_BASKET_POS = 1700,
        LOW_BASKET_SAFETY_POS = 1680,
        HIGH_BASKET_POS = 3400,
        HIGH_BASKET_SAFETY_POS = 3380,
        ABSOLUTE_MAX = 3420,
        ABSOLUTE_MIN = -5;

    public static final int DESTINATION_THRESHOLD = 20;
    public enum StateType {
        TROUGH, TROUGH_SAFETY, DROP_AREA, RAM_BEFORE, RAM_AFTER, BASKET_DEPOSIT, TRANSITION
    }
    private final StateManager<StateType> stateManager;
    private final MotorTransitionState<StateType> transitionState;

    public Lift(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stateManager = new StateManager<>(StateType.TROUGH_SAFETY);

        NothingState<StateType> troughState = new NothingState<>(StateType.TROUGH);
        troughState.addMotor(liftMotor);
        stateManager.addState(StateType.TROUGH, troughState);

        pid = new PIDController(0.01, 0, 0);
        pid.setInputBounds(ABSOLUTE_MIN, ABSOLUTE_MAX);
        pid.setOutputBounds(-1, 1);

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

        this.transitionState = new MotorTransitionState<>(StateType.TRANSITION, liftMotor, DESTINATION_THRESHOLD);
        this.transitionState.setEncoderBounds(ABSOLUTE_MIN, ABSOLUTE_MAX);
        stateManager.addState(StateType.TRANSITION, this.transitionState);

        stateManager.setupStates(robot, stateManager);
    }

    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }

    public Action moveTo(int target) {
        return new TimedAction() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                updateFramesRunning();
                int dif = target - liftMotor.getCurrentPosition();
                liftMotor.setPower(Math.signum(dif) * FULL_POWER);

                return Subsystem.inRange(liftMotor, target, DESTINATION_THRESHOLD);
            }
        };
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
        return robot.isHighRam() ? HIGH_RAM_BEFORE_POS : LOW_RAM_BEFORE_POS;
    }
    public int getRamAfterPos() {
        return robot.isHighRam() ? HIGH_RAM_AFTER_POS : LOW_RAM_AFTER_POS;
    }
    public int getBasketSafetyPos() {
        return robot.isHighDeposit() ? HIGH_BASKET_SAFETY_POS : LOW_BASKET_SAFETY_POS;
    }
    public int getBasketDepositPos() {
        return robot.isHighDeposit() ? HIGH_BASKET_POS : LOW_BASKET_POS;
    }
    public boolean atHighBasket() {
        return stateManager.getActiveStateType() == StateType.BASKET_DEPOSIT &&
                getTransitionState().getGoalStatePosition() == HIGH_BASKET_POS;
    }
    public boolean atLowBasket() {
        return stateManager.getActiveStateType() == StateType.BASKET_DEPOSIT &&
                getTransitionState().getGoalStatePosition() == LOW_BASKET_POS;
    }
    public PIDController getPid() {
        return pid;
    }
}