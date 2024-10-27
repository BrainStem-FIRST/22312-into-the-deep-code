package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.robot.Collector;

public class CollectorAuto extends Collector {

    public CollectorAuto(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
    }

    // hard reset
    // forces hinge up and collector to stop
    public Action resetCollection() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Actions.runBlocking(
                        new ParallelAction(
                            hingeUp(),

                        )
                );
                return true;
            }
        };
    }

    // repeats until block is found
    public Action collectBlock() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                getSpindleMotor().setPower(MAX_SPIN_POWER);
                if (getBlockColor() != BlockColor.NONE) {
                    setCollectState(CollectState.FULL_MAX);
                }
                return getCollectState() == CollectState.COLLECTING;
            }
        };
    }
    // need to call clearSpittingFrames() before calling spitBlock()
    // repeats for SPITTING_FRAME_TIME frames
    public Action spitBlock() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                getSpindleMotor().setPower(-MAX_SPIN_POWER);
                incrementSpittingFrames();
                return getSpittingFrames() < SPITTING_FRAME_TIME;
            }
        };
    }
    // there is a block in the chamber but the spindles still have to spin (at a slower speed) to keep the block from falling out
    // once the collector finishes hinging up, the spindles can stop
    public Action holdBlock() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                getSpindleMotor().setPower(HOLD_SPIN_POWER);
                return getHingeState() != HingeState.UP;
            }
        };
    }
    // repeats until servo reaches up position
    public Action hingeUp() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setHingeServoPosition(HINGE_UP_POSITION);
                if (getHingeServo().getPosition() >= 1)
                    setHingeState(HingeState.UP);
                return getHingeState() != HingeState.UP;
            }
        };
    }
    // repeats until servo reaches down position
    public Action hingeDown() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setHingeServoPosition(HINGE_DOWN_POSITION);
                if (getHingeServo().getPosition() <= 0)
                    setHingeState(HingeState.DOWN);
                return getHingeState() != HingeState.DOWN;
            }
        };
    }
}