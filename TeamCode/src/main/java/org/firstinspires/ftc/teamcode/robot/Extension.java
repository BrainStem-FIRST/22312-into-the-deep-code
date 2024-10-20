package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class Extension extends Subsystem {

    private DcMotorEx extensionMotor;

    public enum State {
        IN, OUT, EXTENDING, RETRACTING
    }
    private State state = State.IN;

    public Extension(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);

        extensionMotor = hwMap.get(DcMotorEx.class, "ExtensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotorEx getExtensionMotor() { return extensionMotor; }

    public State getState() {
        return state;
    }
    public boolean isExtended() { return state == State.OUT; }
    public boolean isRetracted() { return state == State.IN; }
    public boolean isExtending() { return state == State.EXTENDING; }
    public boolean isRetracting() { return state == State.RETRACTING; }
    public void setState(State state) {
        this.state = state;
    }
}