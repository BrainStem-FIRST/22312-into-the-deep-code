package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class Extension extends Subsystem {
    private DcMotorEx extensionMotor;
    private ServoImplEx hingeServo;

    public enum State {
        IN, OUT, EXTENDING, RETRACTING
    }
    private State state = State.IN;

    public Extension(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);

        extensionMotor = hwMap.get(DcMotorEx.class, "ExtensionMotor");
        hingeServo = hwMap.get(ServoImplEx.class, "HingeServo");
    }

    public State getState() {
        return state;
    }
    public void setState(State state) {
        this.state = state;
    }

    private void update() {}
}