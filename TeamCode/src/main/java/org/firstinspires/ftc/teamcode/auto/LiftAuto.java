package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tele.SubsystemTele;

public class LiftAuto extends Lift {

    // TODO: use encoder to find max and min positions of lift
    public static int MAX_TICK_POSITION = 100;
    public static int MIN_TICK_POSITION = 1000;

    HardwareMap hwMap;
    Telemetry telemetry;
    DcMotorEx liftMotor;

    enum LiftStates {
        UP, DOWN, STATIC
    };

    LiftStates liftState;

    public LiftAuto(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftState = LiftStates.STATIC;
    }

    public Action raiseLift() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                liftMotor.setTargetPosition(LiftAuto.MAX_TICK_POSITION);
                return false;
            }
        };
    }

    public Action lowerLift() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                liftMotor.setTargetPosition(LiftAuto.MIN_TICK_POSITION);
                return false;
            }
        };
    }
}