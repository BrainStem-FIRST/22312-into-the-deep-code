package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

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

    public Action collectBlock() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                getSpindleMotor().setPower(MAX_SPIN_POWER);
                return !getFullyCollectedSensor().isPressed();
            }
        };
    }
    public Action hingeUp() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                for (ServoImplEx servo : getHingeServos()) {
                    servo.setPosition(HINGE_UP_POSITION);
                }
                return false;
            }
        };
    }
    public Action hingeDown() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                for (ServoImplEx servo : getHingeServos()) {
                    servo.setPosition(HINGE_DOWN_POSITION);
                }
                return false;
            }
        };
    }
}