package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.hangingStates.HoldHang;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Hanger extends Subsystem<Hanger.StateType> {
    // down refers to the position the hanging goes to after it is on the bar
    public static DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.REVERSE;
    public static int HANG_DOWN_ENCODER = -1300,
            HANG_PARK_ENCODER = 0,
            UP_TICK = 2100,
            DESTINATION_THRESHOLD = 90;
    public static double KP = 0.004, KI = 0.0008;
    public static double HANG_HOLD_POWER = 0;

    public enum StateType {
        FULL_DOWN,
        PARK,
        UP,
        HANG_DOWN,
        TRANSITION
    }
    private boolean movingUp;
    private boolean hangingDown;
    private final MotorTransitionState<StateType> transitionState;
    private final DcMotorEx hangMotor;
    private final PIDController pid;
    public Hanger(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot, StateType.FULL_DOWN);

        hangMotor = hwMap.get(DcMotorEx.class, "HangMotor");
        hangMotor.setDirection(motorDirection);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pid = new PIDController(KP, KI, 0);

        stateManager.addState(StateType.FULL_DOWN, new NothingState<>(StateType.FULL_DOWN));
        stateManager.addState(StateType.PARK, new NothingState<>(StateType.PARK));
        stateManager.addState(StateType.UP, new NothingState<>(StateType.UP));
        stateManager.addState(StateType.HANG_DOWN, new HoldHang());

        transitionState = new MotorTransitionState<>(StateType.TRANSITION, hangMotor, DESTINATION_THRESHOLD, pid);
        stateManager.addState(StateType.TRANSITION, transitionState);

        stateManager.setupStates(robot, stateManager);

        movingUp = true;
        hangingDown = false;
    }

    public DcMotorEx getHangMotor() {
        return hangMotor;
    }
    public MotorTransitionState<StateType> getTransitionState() {
        return transitionState;
    }
    public void setHangMotorPower(double power) {
        Subsystem.setMotorPower(hangMotor, power);
    }

    @Override
    public void update(double dt) {
        // hanging down if need
        if(hangingDown && hangMotor.getCurrentPosition() > Hanger.HANG_DOWN_ENCODER + Hanger.DESTINATION_THRESHOLD) {
            Subsystem.setMotorPower(hangMotor, -1);
            movingUp = false;
        }
        // setting up for hang if need
        else if(movingUp && hangMotor.getCurrentPosition() < Hanger.UP_TICK - Hanger.DESTINATION_THRESHOLD) {
            Subsystem.setMotorPower(hangMotor, 1);
            hangingDown = false;
        }
        // resting if need
        else {
            Subsystem.setMotorPower(hangMotor, 0);
            hangingDown = false;
            movingUp = false;
        }

        // old code
        //stateManager.update(dt);
    }
    public Action moveTo(int target) {
        return telemetryPacket -> {
            Subsystem.setMotorPosition(hangMotor, Hanger.HANG_PARK_ENCODER);
            return Subsystem.inRange(hangMotor, target, DESTINATION_THRESHOLD);
        };
    }
    public void startMovingUp() {
        movingUp = true;
        hangingDown = false;
    }
    public void startHangingDown() {
        hangingDown = true;
        movingUp = false;
    }
}
