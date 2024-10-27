package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends Subsystem {

    // TODO: use encoder to find max and min positions of lift
    public static int MAX_TICK_POSITION = 100;
    public static int MIN_TICK_POSITION = 1000;

    public enum State {
        UP, DOWN, STATIC
    };
    private State state;

    private final DcMotorEx liftMotor;

    public Lift(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        state = State.STATIC;
    }

    public State getState() {
        return state;
    }
    public void setState(State state) {
        this.state = state;
    }
    public DcMotorEx getLiftMotor() {
        return liftMotor;
    }
}

