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
    public static final int MIN_TICK = 935, MAX_TICK = 2445;
    // TODO: FIND CORRECT TRANSITION TIMES FOR ARM
    // if times r wrong then optimizations will be messed up
    public static final double TRANSFER_TO_BASKET_SAFETY_TIME = 0.3, BASKET_SAFETY_TO_BLOCK_DROP_TIME = 0.2, BLOCK_DROP_TO_UP_TIME = 0.05, TRANSFER_TO_BLOCK_DROP_TIME = 0.1, BLOCK_DROP_TO_SPECIMEN_HANG_TIME = 0.2, SPECIMEN_HANG_TO_UP_TIME = 0.1, SPECIMEN_HANG_TO_TRANSFER_TIME = 0.3;
    public static final double TRANSFER_POS = 0.01, BLOCK_DROP_POS = 0.33, SPECIMEN_HANG_POS = 0.99, BASKET_SAFETY_POS = 0.8;
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

        transitionState = new ServoTransitionState<>(StateType.TRANSITION, armServo);
        stateManager.addState(StateType.TRANSITION, transitionState);

        stateManager.setupStates(robot, stateManager);
    }

    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }

    public Action rotateTo(double pos) {
        return new TimedAction() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                updateFramesRunning();
                armServo.setPosition(pos);
                return getTime() > 0.3;
            }
        };
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
