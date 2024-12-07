package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.PIDController;


@Config
public class Lift extends Subsystem {
    private final DcMotorEx liftMotor;
    private final PIDController pid;
    public static final double NORMAL_KP = 0.003, NORMAL_KI = 0.0008, NORMAL_KD = 0, LOWERING_KP = 0.002, LOWERING_KI = 0;
    public static int ABSOLUTE_MIN = -50, TROUGH_POS = -5;
    public static int TROUGH_SAFETY_POS = 350, // position where arm can safely raise without colliding with collector
        DROP_AREA_POS = 50, // position where grabber can grab onto specimen
        DROP_AREA_AFTER_POS = 200, // position to go to after grabber has specimen (to clear specimen off wall)
        LOW_RAM_BEFORE_POS = 320, // position to go to to setup for low bar ram
        LOW_RAM_AFTER_POS = 595, // position to go to after ramming low bar
        HIGH_RAM_BEFORE_POS = 1300, // position to go to to setup for high bar ram
        HIGH_RAM_AFTER_POS = 1860, // position to go to after ramming high bar

        LOW_BASKET_POS = 1940, // position to go to so arm and grabber can deposit block on low basket
        LOW_BASKET_SAFETY_POS = 1360, // position where arm can start rotating into position to deposit on low basket
        AUTO_HIGH_BASKET_POS = 3300,
        HIGH_BASKET_POS = 3400, // position to go to so arm and grabber can deposit block on high basket
        HIGH_BASKET_SAFETY_POS = 2800, // position where arm can start rotating into position to deposit on high basket
        ABSOLUTE_MAX = 3420;

    public static int DESTINATION_THRESHOLD = 30, // threshold in which I consider a lift transition done during tele
        AUTO_DESTINATION_THRESHOLD = 15, // threshold in which I consider a lift transition done during auto
        AUTO_TO_POSITION_THRESHOLD = 300; // when lift distance from goal encoder is smaller than this, I set lift to runToTargetPosition
    public enum StateType {
        TROUGH, TROUGH_SAFETY, DROP_AREA, DROP_AREA_AFTER, RAM_BEFORE, RAM_AFTER, BASKET_DEPOSIT, TRANSITION
    }
    private final StateManager<StateType> stateManager;
    private final MotorTransitionState<StateType> transitionState;

    public Lift(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDController(NORMAL_KP, NORMAL_KI, NORMAL_KD);
        pid.setInputBounds(ABSOLUTE_MIN, ABSOLUTE_MAX);
        pid.setOutputBounds(-1, 1);

        stateManager = new StateManager<>(StateType.TROUGH_SAFETY);

        stateManager.addState(StateType.TROUGH, new NothingState<>(StateType.TROUGH, liftMotor));
        stateManager.addState(StateType.TROUGH_SAFETY, new NothingState<>(StateType.TROUGH_SAFETY, liftMotor));
        stateManager.addState(StateType.DROP_AREA, new NothingState<>(StateType.DROP_AREA, liftMotor));
        stateManager.addState(StateType.DROP_AREA_AFTER, new NothingState<>(StateType.DROP_AREA_AFTER, liftMotor));
        stateManager.addState(StateType.RAM_BEFORE, new NothingState<>(StateType.RAM_BEFORE, liftMotor));
        stateManager.addState(StateType.RAM_AFTER, new NothingState<>(StateType.RAM_AFTER, liftMotor));
        stateManager.addState(StateType.BASKET_DEPOSIT, new NothingState<>(StateType.BASKET_DEPOSIT, liftMotor));

        this.transitionState = new MotorTransitionState<>(StateType.TRANSITION, liftMotor, DESTINATION_THRESHOLD, pid);
        this.transitionState.setEncoderBounds(ABSOLUTE_MIN, ABSOLUTE_MAX);
        stateManager.addState(StateType.TRANSITION, this.transitionState);

        stateManager.setupStates(robot, stateManager);
    }

    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }
    private void updateLiftAutoMovement(int target) {
        if(Subsystem.inRange(liftMotor, target, Lift.AUTO_TO_POSITION_THRESHOLD))
            Subsystem.setMotorPosition(liftMotor, target);
        else
            Subsystem.setMotorPower(liftMotor, Range.clip(target - liftMotor.getCurrentPosition(), -1, 1));
    }
    public Action moveTo(int target) {
        return (@NonNull TelemetryPacket telemetryPacket) -> {
            Subsystem.setMotorPosition(liftMotor, target);
            //updateLiftAutoMovement(target);
            return !Subsystem.inRange(liftMotor, target, AUTO_DESTINATION_THRESHOLD);
        };
    }
    public Action moveTo(int target, int posToPass) {
        return (@NonNull TelemetryPacket telemetryPacket) -> {
            Subsystem.setMotorPosition(liftMotor, target);
            //updateLiftAutoMovement(target);
            return !Subsystem.inRange(liftMotor, posToPass, AUTO_DESTINATION_THRESHOLD)
                    && !Subsystem.inRange(liftMotor, target, AUTO_DESTINATION_THRESHOLD);
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