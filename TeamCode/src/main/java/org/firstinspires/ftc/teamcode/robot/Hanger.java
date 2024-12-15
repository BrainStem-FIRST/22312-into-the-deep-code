package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;

@Config
public class Hanger extends Subsystem<Hanger.StateType> {
    // down refers to the position the hanging goes to after it is on the bar
    public static DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.REVERSE;
    public static int FULL_DOWN_ENCODER = 0,
            HANG_PARK_ENCODER = 4950,
            UP_TICK = 4950,
            DESTINATION_THRESHOLD = 70;
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
    private final MotorTransitionState<StateType> transitionState;
    private final DcMotorEx hangMotor;
    public Hanger(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot, StateType.FULL_DOWN);

        hangMotor = hwMap.get(DcMotorEx.class, "HangMotor");
        hangMotor.setDirection(motorDirection);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stateManager.addState(StateType.FULL_DOWN, new NothingState<>(StateType.FULL_DOWN));
        stateManager.addState(StateType.PARK, new NothingState<>(StateType.PARK));
        stateManager.addState(StateType.UP, new NothingState<>(StateType.UP));

        transitionState = new MotorTransitionState<>(StateType.TRANSITION, hangMotor, DESTINATION_THRESHOLD);
        stateManager.addState(StateType.TRANSITION, transitionState);

        stateManager.setupStates(robot, stateManager);

        movingUp = true;
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
        // setting up for hang if need
        if(movingUp && hangMotor.getCurrentPosition() < Hanger.UP_TICK - Hanger.DESTINATION_THRESHOLD) {
            Subsystem.setMotorPower(hangMotor, 1);
        }
        // resting if need
        else {
            Subsystem.setMotorPower(hangMotor, 0);
            movingUp = false;
        }

        // old code
        //stateManager.update(dt);
    }
    private Action moveToParkSetPower() {
        return telemetryPacket -> {
            Subsystem.setMotorPower(hangMotor, 1);
            Log.d("hang encoder", hangMotor.getCurrentPosition() + "");

            return hangMotor.getCurrentPosition() < HANG_PARK_ENCODER - DESTINATION_THRESHOLD;
        };
    }
    public Action moveToPark() {
        return new SequentialAction(
                moveToParkSetPower(),
                telemetryPacket -> {
                    Log.d("stopping hang motor", "");
                    Subsystem.setMotorPower(hangMotor, 0);
                    return false;
                }
        );
    }
    public void startMovingUp() {
        movingUp = true;
    }
}
