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
public class Grabber extends Subsystem<Grabber.StateType> {
    public static int MIN_TICK = 1210, MAX_TICK = 2400;
    public static double CLOSE_POS = 0.04, OPEN_POS = 0.99;
    public static double FULL_ROTATION_TIME = 0.19;
    private BlockColor blockColorHeld;
    private boolean hasSpecimen;

    public enum StateType {
        OPEN, CLOSED, TRANSITION
    }
    private final ServoTransitionState<StateType> transitionState;
    private final ServoImplEx grabServo;


    public Grabber(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot, StateType.OPEN);

        grabServo = hwMap.get(ServoImplEx.class, "LiftGrabServo");
        grabServo.setPwmRange(new PwmControl.PwmRange(MIN_TICK, MAX_TICK));

        blockColorHeld = BlockColor.NONE;
        hasSpecimen = false;

        stateManager.addState(StateType.OPEN, new NothingState<>(StateType.OPEN));
        stateManager.addState(StateType.CLOSED, new NothingState<>(StateType.CLOSED));
        transitionState = new ServoTransitionState<>(StateType.TRANSITION, grabServo, FULL_ROTATION_TIME);
        stateManager.addState(StateType.TRANSITION, transitionState);

        stateManager.setupStates(robot, stateManager);
    }
    @Override
    public void update(double dt) {
        stateManager.update(dt);
    }

    public Action open() {
        return new SequentialAction(
                openServo(),
                new SleepAction(FULL_ROTATION_TIME)
        );
    }
    public Action close() {
        return new SequentialAction(
                closeServo(),
                new SleepAction(FULL_ROTATION_TIME)
        );
    }
    private Action openServo() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                grabServo.setPosition(OPEN_POS);
                return false;
            }
        };
    }
    private Action closeServo() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                grabServo.setPosition(CLOSE_POS);
                return false;
            }
        };
    }
    public ServoTransitionState<StateType> getTransitionState() {
        return transitionState;
    }
    public ServoImplEx getGrabServo() {
        return grabServo;
    }
    public BlockColor getBlockColorHeld() {
        return blockColorHeld;
    }
    public boolean hasBlock() {
        return blockColorHeld != BlockColor.NONE;
    }
    public boolean hasSpecimen() {
        return hasSpecimen;
    }

    public void setBlockColorHeld(BlockColor blockColorHeld) {
        this.blockColorHeld = blockColorHeld;
        hasSpecimen = false;
    }
    public void setHasSpecimen(boolean hasSpecimen) {
        if(hasSpecimen)
            setBlockColorHeld(robot.getColorFromAlliance());
        else
            setBlockColorHeld(BlockColor.NONE);
        this.hasSpecimen = hasSpecimen;
    }
}
