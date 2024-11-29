package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystem {
    /**
     * standardized function to set motor to go to target position
     * @param motor motor to run
     * @param position target position
     */
    public static void setMotorPosition(DcMotorEx motor, int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    /**
     * standardized function to set motor to run at target power
     * @param motor motor to run
     * @param power target power
     */
    public static void setMotorPower(DcMotorEx motor, double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    /**
     * checks for if motor within threshold of target position
     * @param motor motor to check
     * @param target target to compare to
     * @param threshold threshold that determines if motor in range
     * @return whether motor in range or not
     */
    public static boolean inRange(DcMotorEx motor, int target, int threshold) {
        return Math.abs(motor.getCurrentPosition() - target) <= threshold;
    }

    protected HardwareMap hwMap;
    protected Telemetry telemetry;
    private final AllianceColor allianceColor;
    private final BrainSTEMRobot robot;

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