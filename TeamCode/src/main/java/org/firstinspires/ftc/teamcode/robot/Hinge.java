package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.TimedAction;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.ServoTransitionState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class Hinge extends Subsystem {
    public static final int HINGE_UP_TICK = 2200, HINGE_DOWN_TICK = 1480;
    public static final double HINGE_UP_POSITION = 0.01, HINGE_DOWN_POSITION = 0.99, HINGE_MIDDLE_POSITION = 0.5;
    public static double HINGE_TIME = 0.5;

    public enum StateType {
        UP,
        MIDDLE,
        DOWN,
        TRANSITION
    }

    private final StateManager<StateType> stateManager;
    private final ServoTransitionState<StateType> transitionState;
    private final ServoImplEx hingeServo;
    public Hinge(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        hingeServo = hwMap.get(ServoImplEx.class, "CollectHingeServo");
        hingeServo.setPwmRange(new PwmControl.PwmRange(HINGE_UP_TICK, HINGE_DOWN_TICK));

        stateManager = new StateManager<>(StateType.UP);

        stateManager.addState(StateType.UP, new NothingState<>(StateType.UP));
        stateManager.addState(StateType.DOWN, new NothingState<>(StateType.DOWN));
        stateManager.addState(StateType.MIDDLE, new NothingState<>(StateType.MIDDLE));

        transitionState = new ServoTransitionState<>(StateType.TRANSITION, hingeServo, HINGE_TIME);
        stateManager.addState(StateType.TRANSITION, transitionState);

        stateManager.setupStates(getRobot(), stateManager);
    }
    public StateManager<StateType> getStateManager() {
        return stateManager;
    }

    public ServoTransitionState<StateType> getTransitionState() {
        return transitionState;
    }
    public ServoImplEx getHingeServo() { return hingeServo; }
    public void setHingeServoPosition(double position) {
        hingeServo.setPosition(position);
    }


    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }


    public Action hingeUpAction() {
        return new TimedAction() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setHingeServoPosition(HINGE_UP_POSITION);
                return getTime() < HINGE_TIME;
            }
        };
    }
    public Action hingeDownAction() {
        return new TimedAction() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setHingeServoPosition(HINGE_DOWN_POSITION);
                return getTime() < HINGE_TIME;
            }
        };
    }
}
