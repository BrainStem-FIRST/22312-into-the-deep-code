package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.BrainSTEMRobotAuto;

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
    private final BrainSTEMRobot robot;
    private final BrainSTEMRobotAuto robotAuto;
    private final Gamepad gamepad1, gamepad2;

    public Subsystem(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
        this.robot = robot;
        robotAuto = robot.getClass() == BrainSTEMRobotAuto.class ? (BrainSTEMRobotAuto) robot : null;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public AllianceColor getAllianceColor() {
        return allianceColor;
    }
    public BrainSTEMRobot getRobot() {
        return robot;
    }
    public BrainSTEMRobotAuto getRobotAuto() {
        return robotAuto;
    }
    public Gamepad getGamepad1() {
        return gamepad1;
    }
    public Gamepad getGamepad2() {
        return gamepad2;
    }
    public abstract void update(double dt);
}