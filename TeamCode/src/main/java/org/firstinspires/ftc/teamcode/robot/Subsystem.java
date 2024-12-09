package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public abstract class Subsystem<StateType extends Enum<StateType>> {
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

    /**
     * @param start pwm of starting position
     * @param end pwm of ending position
     * @param fullRotationTime time to rotate from 0.01 pwm to 0.98 pwm
     * @return returns time to travel from start to end
     */
    public static double getServoTime(double start, double end, double fullRotationTime) {
        // dist / entire dist = time / entire time
        // time = dist / entire dist * entire time
        return Math.abs(end - start) / 0.98 * fullRotationTime;
    }

    protected HardwareMap hwMap;
    protected Telemetry telemetry;
    protected final AllianceColor allianceColor;
    protected final BrainSTEMRobot robot;

    protected final StateManager<StateType> stateManager;

    public Subsystem(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot, StateType defaultState) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
        this.robot = robot;
        stateManager = new StateManager<>(defaultState);
    }
    public BrainSTEMRobot getRobot() {
        return robot;
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }
    public abstract void update(double dt);
}