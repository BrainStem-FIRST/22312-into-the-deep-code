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
import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.ServoTransitionState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;


@Config
public class Arm extends Subsystem<Arm.StateType> {
    public static final int MIN_TICK = 935, MAX_TICK = 2450;


    // describes time needed for arm to rotate from 0.01 pwm to 0.99 pwm
    // not sure if it fully works tho bc the angular velocity will change depending on the load of the arm
    public static double TRANSFER_TO_KNOCK_BLOCK_TIME = 0.14,
        TRANSFER_TO_BASKET_SAFETY_TIME = 0.62,
        BASKET_SAFETY_TO_BASKET_DROP_TIME = 0.35,
        BASKET_DROP_TO_DROP_OFF_TIME = 0.5,
        BASKET_DROP_TO_UP_TIME = 0.18,
        UP_TO_BASKET_SAFETY_TIME = 0.42,
        SPECIMEN_HANG_TO_UP_TIME = 0.18,
        TRANSFER_TO_DROP_AREA_TIME = 0.28,
        DROP_AREA_TO_RAM_TIME = 0.66,
        UP_TO_TRANSFER_TIME = 0.55;
    public static final double TRANSFER_POS = 0.01,
        KNOCK_BLOCK_POS = 0.15,
        DROP_OFF_POS = 0.33,
        BASKET_DROP_POS = 0.30,
        UP_POS = 0.67,
        SPECIMEN_HANG_POS = 0.99,
        BASKET_SAFETY_POS = 0.85;
    public enum StateType {
        TRANSFER,
        KNOCK_BLOCK,
        DROP_OFF,
        BASKET_DROP,
        UP,
        BASKET_SAFETY,
        SPECIMEN_HANG,
        TRANSITION
    }
    private final ServoTransitionState<StateType> transitionState;
    private final ServoImplEx armServo;

    public Arm(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot, StateType.TRANSFER);

        armServo = hwMap.get(ServoImplEx.class, "LiftArmServo");
        armServo.setPwmRange(new PwmControl.PwmRange(MIN_TICK, MAX_TICK));

        stateManager.addState(StateType.TRANSFER, new NothingState<>(StateType.TRANSFER));
        stateManager.addState(StateType.KNOCK_BLOCK, new NothingState<>(StateType.KNOCK_BLOCK));
        stateManager.addState(StateType.DROP_OFF, new NothingState<>(StateType.DROP_OFF));
        stateManager.addState(StateType.BASKET_DROP, new NothingState<>(StateType.BASKET_DROP));
        stateManager.addState(StateType.UP, new NothingState<>(StateType.UP));
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

    public Action rotateTo(double pos, double timeDone) {
        telemetry.update();
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
    public ServoImplEx getArmServo() {
        return armServo;
    }
    public ServoTransitionState<StateType> getTransitionState() {
        return transitionState;
    }
}
