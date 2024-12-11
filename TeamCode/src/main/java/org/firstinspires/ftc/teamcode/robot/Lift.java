package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.util.PIDController;


@Config
public class Lift extends Subsystem<Lift.StateType> {
    public static int DESTINATION_THRESHOLD = 40, // threshold in which I consider a lift transition done during tele
            AUTO_DESTINATION_THRESHOLD = 50; // threshold in which I consider a lift transition done during auto
    public static int ABSOLUTE_MIN = -50,
        TROUGH_POS = 10,
        AUTO_TROUGH_POS = TROUGH_POS - AUTO_DESTINATION_THRESHOLD,
        KNOCK_BLOCK_POS = 170,
        TROUGH_SAFETY_POS = 450, // position where arm can safely raise without colliding with collector
        DROP_AREA_POS = 50, // position where grabber can grab onto specimen
        DROP_AREA_AFTER_POS = 200, // position to go to after grabber has specimen (to clear specimen off wall)
        LOW_RAM_BEFORE_POS = 320, // position to go to to setup for low bar ram
        LOW_RAM_AFTER_POS = 595, // position to go to after ramming low bar
        HIGH_RAM_BEFORE_POS = 1300, // position to go to to setup for high bar ram
        HIGH_RAM_AFTER_POS = 1820, // position to go to after ramming high bar

        LOW_BASKET_POS = 1940, // position to go to so arm and grabber can deposit block on low basket
        LOW_BASKET_SAFETY_POS = 1360, // position where arm can start rotating into position to deposit on low basket
        HIGH_BASKET_POS = 3400, // position to go to so arm and grabber can deposit block on high basket
        HIGH_BASKET_SAFETY_POS = 2730, // position where arm can start rotating into position to deposit on high basket
        ABSOLUTE_MAX = 3420;

    public enum StateType {
        TROUGH, KNOCK_BLOCK, TROUGH_SAFETY, DROP_AREA, DROP_AREA_AFTER, RAM_BEFORE, RAM_AFTER, BASKET_DEPOSIT, TRANSITION
    }
    private final MotorTransitionState<StateType> transitionState;

    public static double ZERO_KI = 0, SMALL_TRANSITION_KI = 0.0008;
    public static double KP = 0.003, SMALL_TRANSITION_KP = 0.002, ZERO_KD = 0;
    private final DcMotorEx liftMotor;
    private final PIDController pid;

    public Lift(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot, StateType.TROUGH_SAFETY);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDController(KP, ZERO_KI, ZERO_KD);
        pid.setInputBounds(ABSOLUTE_MIN, ABSOLUTE_MAX);
        pid.setOutputBounds(-1, 1);

        stateManager.addState(StateType.TROUGH, new NothingState<>(StateType.TROUGH, liftMotor));
        stateManager.addState(StateType.KNOCK_BLOCK, new NothingState<>(StateType.KNOCK_BLOCK));
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
    public void updateLiftAutoPidMovement(int target) {
        pid.setkI(SMALL_TRANSITION_KI);
        pid.setTarget(target);
        /*
        telemetry.addData("lift pid target", pid.getTarget());
        telemetry.addData("lift power", liftMotor.getPower());
        telemetry.addData("lift position", liftMotor.getCurrentPosition());
        telemetry.addData("kP", pid.getkP());
        telemetry.addData("kI", pid.getkI());
        telemetry.addData("kD", pid.getkD());
        telemetry.update();
         */
        Subsystem.setMotorPower(liftMotor, pid.update(liftMotor.getCurrentPosition()));
    }
    public Action moveTo(int target) {
        return (@NonNull TelemetryPacket telemetryPacket) -> {
            updateLiftAutoPidMovement(target);
            return !Subsystem.inRange(liftMotor, target, AUTO_DESTINATION_THRESHOLD);
        };
    }
    public Action moveTo(int target, int posToPass) {
        pid.reset();
        return (@NonNull TelemetryPacket telemetryPacket) -> {
            updateLiftAutoPidMovement(target);
            return !Subsystem.inRange(liftMotor, target, AUTO_DESTINATION_THRESHOLD)
                    && !Subsystem.inRange(liftMotor, posToPass, AUTO_DESTINATION_THRESHOLD);
        };
    }
    public Action moveToWithoutPid(int target) {
        return (@NonNull TelemetryPacket telemetryPacket) -> {
            Subsystem.setMotorPosition(liftMotor, target);
            /*
            telemetry.addData("lift target", liftMotor.getTargetPosition());
            telemetry.addData("lift encoder", liftMotor.getCurrentPosition());
            telemetry.update();
            */

            return !Subsystem.inRange(liftMotor, target, AUTO_DESTINATION_THRESHOLD);
        };
    }
    public Action moveToWithoutPid(int target, int posToPass) {
        return (@NonNull TelemetryPacket telemetryPacket) -> {

            Subsystem.setMotorPosition(liftMotor, target);

            /*
            telemetry.addData("lift target", liftMotor.getTargetPosition());
            telemetry.addData("lift encoder", liftMotor.getCurrentPosition());
            telemetry.update();
             */

            return !Subsystem.inRange(liftMotor, target, AUTO_DESTINATION_THRESHOLD)
                    && !Subsystem.inRange(liftMotor, posToPass, AUTO_DESTINATION_THRESHOLD);
        };
    }

    public DcMotorEx getLiftMotor() {
        return liftMotor;
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