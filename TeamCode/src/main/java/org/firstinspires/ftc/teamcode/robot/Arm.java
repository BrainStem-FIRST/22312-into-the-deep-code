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
import org.firstinspires.ftc.teamcode.robotStates.ServoTransitionState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class Arm extends Subsystem {
    public static final int MIN_TICK = 935, MAX_TICK = 2450;

    // TODO: find FULL_ROTATION_TIME;
    // if times r wrong then optimizations will be messed up
    public static final double FULL_ROTATION_TIME = 0.5;
    public static final double TRANSFER_POS = 0.01, BLOCK_DROP_POS = 0.33, UP_POS = 0.67, SPECIMEN_HANG_POS = 0.99, BASKET_SAFETY_POS = 0.85;
    public static final double BLOCK_DROP_TO_UP_TIME = Subsystem.getServoTime(BLOCK_DROP_POS, UP_POS, FULL_ROTATION_TIME), SPECIMEN_HANG_TO_UP_TIME = Subsystem.getServoTime(SPECIMEN_HANG_POS, UP_POS, FULL_ROTATION_TIME);
    public enum StateType {
        TRANSFER, BLOCK_DROP, SPECIMEN_HANG, BASKET_SAFETY, TRANSITION
    }
    private final StateManager<StateType> stateManager;
    private final ServoTransitionState<StateType> transitionState;
    private final ServoImplEx armServo;

    public Arm(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);

        armServo = hwMap.get(ServoImplEx.class, "LiftArmServo");
        armServo.setPwmRange(new PwmControl.PwmRange(MIN_TICK, MAX_TICK));

        stateManager = new StateManager<>(Arm.StateType.TRANSFER);

        stateManager.addState(StateType.TRANSFER, new NothingState<>(StateType.TRANSFER));
        stateManager.addState(StateType.BLOCK_DROP, new NothingState<>(StateType.BLOCK_DROP));
        stateManager.addState(StateType.BASKET_SAFETY, new NothingState<>(StateType.BASKET_SAFETY));
        stateManager.addState(StateType.SPECIMEN_HANG, new NothingState<>(StateType.SPECIMEN_HANG));

        transitionState = new ServoTransitionState<>(StateType.TRANSITION, armServo, FULL_ROTATION_TIME);
        stateManager.addState(StateType.TRANSITION, transitionState);

        stateManager.setupStates(robot, stateManager);
    }

    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }

    public Action rotateTo(double pos, double timeDone) {
        return new TimedAction() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                updateFramesRunning();
                armServo.setPosition(pos);
                return getTime() > timeDone;
            }
        };
    }
    public double timeToRotateTo(double goalPos) {
        return Subsystem.getServoTime(armServo.getPosition(), goalPos, FULL_ROTATION_TIME);
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }
    public ServoImplEx getArmServo() {
        return armServo;
    }
    public ServoTransitionState<StateType> getTransitionState() {
        return transitionState;
    }
}
