package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.BrainSTEMRobotAuto;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public abstract class Subsystem {
    public static void setMotorPosition(DcMotorEx motor, int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }
    public static void setMotorPower(DcMotorEx motor, double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    protected HardwareMap hwMap;
    protected Telemetry telemetry;
    private final AllianceColor allianceColor;
    private BrainSTEMRobot robot;

    public Subsystem(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
        this.robot = robot;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }
    public BrainSTEMRobot getRobot() {
        return robot;
    }
    public abstract void update(double dt);
}