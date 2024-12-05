package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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


    // describes time needed for arm to rotate from 0.01 pwm to 0.99 pwm
    // not sure if it fully works tho bc the angular velocity will change depending on the load of the arm
    public static final double FULL_ROTATION_TIME = 0.8;
    public static final double TRANSFER_POS = 0.01, DROP_OFF_POS = 0.33, BASKET_DROP_POS = 0.28, UP_POS = 0.67, SPECIMEN_HANG_POS = 0.99, BASKET_SAFETY_POS = 0.85;
    public static final double SPECIMEN_HANG_TO_UP_TIME = Subsystem.getServoTime(SPECIMEN_HANG_POS, UP_POS, FULL_ROTATION_TIME);

    public enum StateType {
        TRANSFER, DROP_OFF, BASKET_DROP, UP, BASKET_SAFETY, SPECIMEN_HANG, TRANSITION
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
        stateManager.addState(StateType.DROP_OFF, new NothingState<>(StateType.DROP_OFF));
        stateManager.addState(StateType.BASKET_DROP, new NothingState<>(StateType.BASKET_DROP));
        stateManager.addState(StateType.UP, new NothingState<>(StateType.UP));
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
        return new SequentialAction(
                rotateServo(pos),
                new SleepAction(timeDone)
        );
    }
    private Action rotateServo(double pos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armServo.setPosition(pos);
                return false;
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
