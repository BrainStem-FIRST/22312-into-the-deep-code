package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Subsystem {

    protected HardwareMap hwMap;
    protected Telemetry telemetry;
    private final AllianceColor allianceColor;

    public Subsystem(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
    }
    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    protected void setMotorPosition(DcMotorEx motor, int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }
    protected void setMotorPower(DcMotorEx motor, double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }
}