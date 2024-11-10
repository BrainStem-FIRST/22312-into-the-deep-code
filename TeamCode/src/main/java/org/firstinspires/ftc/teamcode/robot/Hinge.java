package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.TimedAction;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.hingeStates.HingeDownState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.hingeStates.HingeUpState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class Hinge extends Subsystem {
    public static final int HINGE_UP_TICK = 2213, HINGE_DOWN_TICK = 1726;
    public static final double HINGE_UP_POSITION = 0.01, HINGE_DOWN_POSITION = 0.99, HINGE_THRESHOLD = 0.05;
    public static double HINGE_DOWN_TIME = 0.5, HINGE_UP_TIME = 0.5;

    public enum StateType {
        UP,
        HINGING_DOWN,
        HINGING_UP,
        DOWN
    }

    private final StateManager<StateType> stateManager;
    private final ServoImplEx hingeServo;
    public Hinge(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        hingeServo = hwMap.get(ServoImplEx.class, "CollectHingeServo");
        hingeServo.setPwmRange(new PwmControl.PwmRange(HINGE_UP_TICK, HINGE_DOWN_TICK));

        stateManager = new StateManager<>(StateType.UP);
        stateManager.addState(StateType.UP, new NothingState<>(StateType.UP));
        stateManager.addState(StateType.HINGING_UP, new HingeUpState());
        stateManager.addState(StateType.DOWN, new NothingState<>(StateType.DOWN));
        stateManager.addState(StateType.HINGING_DOWN, new HingeDownState());
        stateManager.setupStates(getRobot(), stateManager);
    }
    public StateManager<StateType> getStateManager() {
        return stateManager;
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
                return getTime() >= HINGE_UP_TIME;
            }
        };
    }
    public Action hingeDownAction() {
        return new TimedAction() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setHingeServoPosition(HINGE_DOWN_POSITION);
                return getTime() >= HINGE_DOWN_TIME;
            }
        };
    }
}
