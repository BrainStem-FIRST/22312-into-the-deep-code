package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Lift;

public class LiftAuto extends Lift {

    public LiftAuto(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);
        getLiftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public Action raiseLift() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                getLiftMotor().setTargetPosition(LiftAuto.MAX_TICK_POSITION);
                return false;
            }
        };
    }

    public Action lowerLift() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                getLiftMotor().setTargetPosition(LiftAuto.MIN_TICK_POSITION);
                return false;
            }
        };
    }
}