package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.TimedAction;
import org.firstinspires.ftc.teamcode.robotStates.MotorTransitionState;
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.ServoTransitionState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

@Config
public class Hinge extends Subsystem {
    public static int AUTO_SHAKE_FRAMES = 10;
    private boolean isFullyDown;
    public static double HINGE_SHAKE_DOWN_POSITION = 0.6;
    public static int HINGE_UP_TICK = 2250, HINGE_DOWN_TICK = 1480;
    public static double HINGE_UP_POSITION = 0.01, HINGE_DOWN_POSITION = 0.99, HINGE_MIDDLE_POSITION = 0.65;
    public static double HINGE_DOWN_TIME = 0.2, HINGE_UP_TIME = 0.3;

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

        transitionState = new ServoTransitionState<>(StateType.TRANSITION, hingeServo);
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

    // these functions do not override hinge transition if it is already in transition
    public void goToHingeUpState() {
        if (stateManager.getActiveStateType() == StateType.UP)
            return;
        double time =  HINGE_UP_TIME;
        if (stateManager.getActiveStateType() == StateType.MIDDLE)
            time *= 0.5;
        transitionState.setGoalState(Hinge.HINGE_UP_POSITION, Hinge.StateType.UP, time);

    }
    public void goToHingeMiddleState() {
        if (stateManager.getActiveStateType() == StateType.MIDDLE)
            return;

        double time = HINGE_UP_TIME * 0.5;
        if (stateManager.getActiveStateType() == StateType.UP)
            time = HINGE_DOWN_TIME * 0.5;
        transitionState.setGoalState(Hinge.HINGE_MIDDLE_POSITION, Hinge.StateType.MIDDLE, time);
    }
    public void goToHingeDownState() {
        if (stateManager.getActiveStateType() == StateType.DOWN)
            return;

        double time = HINGE_DOWN_TIME;
        if (stateManager.getActiveStateType() == StateType.MIDDLE)
            time *= 0.5;

        transitionState.setGoalState(Hinge.HINGE_DOWN_POSITION, Hinge.StateType.DOWN, time);
    }

    private Action hingeServoUp() {
        return telemetryPacket -> {
            setHingeServoPosition(HINGE_UP_POSITION);
            return false;
        };
    }
    private Action hingeServoDown() {
        return telemetryPacket -> {
            setHingeServoPosition(HINGE_DOWN_POSITION);
            return false;
        };
    }
    public Action hingeUpAction() {
        return new SequentialAction(
                hingeServoUp(),
                new SleepAction(HINGE_UP_TIME)
        );
    }
    public Action hingeDownAction() {
        return new SequentialAction(
                hingeServoDown(),
                new SleepAction(HINGE_DOWN_TIME)
        );
    }

    public Action shakeHingeDown() {
        return new TimedAction() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                updateFramesRunning();
                if (getFramesRunning() % AUTO_SHAKE_FRAMES == 0) {
                    isFullyDown = !isFullyDown;

                    if (isFullyDown)
                        setHingeServoPosition(HINGE_DOWN_POSITION);
                    else
                        setHingeServoPosition(HINGE_SHAKE_DOWN_POSITION);
                }
                return getRobot().getCollector().getBlockColorSensor().getRawBlockColor() == BlockColor.NONE;
            }
        };
    }
}
