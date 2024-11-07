package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ParallelAction;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public class CollectorAuto extends Collector {

    public CollectorAuto(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobotAuto robot) {
        super(hwMap, telemetry, allianceColor, robot);
    }

    public Action hingeAction(int hingeTick) {
        return new Action() {
            @Override
            public boolean run() {
                setHingeServoPosition(Collector.HINGE_UP_POSITION);
                return Math.abs(getHingeServo().getPosition() - Collector.HINGE_UP_POSITION) < Collector.HINGE_THRESHOLD;
            }
        };
    }
}