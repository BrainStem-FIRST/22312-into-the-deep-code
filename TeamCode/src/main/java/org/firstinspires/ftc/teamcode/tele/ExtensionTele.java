package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class ExtensionTele extends Extension {

    final public static int EXTENDED_POSITION = 100;
    final public static int RETRACTED_POSITION = 0;
    final public static double EXTENDING_POWER = 0.2;
    final public static double RETRACTING_POWER = 0.2;
    private PIDController pidController;

    public ExtensionTele(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);

        setState(State.IN);

        // TODO: FINE TUNE PID
        pidController = new PIDController(0.5, 0, 0);
    }
    public void update() {
        switch (getState()) {
            case IN: updateRetracted(); break;
            case OUT: updateExtended(); break;
            case EXTENDING: updateExtending(); break;
            case RETRACTING: updateRetracting(); break;
            default: break;
        }
    }

    private void updateExtending() {
        setMotorPosition(getExtensionMotor(), EXTENDED_POSITION);
        if (getExtensionMotor().getCurrentPosition() >= EXTENDED_POSITION) {
            setState(State.OUT);
        }
    }
    private void updateRetracting() {
        setMotorPosition(getExtensionMotor(), RETRACTED_POSITION);
        if (getExtensionMotor().getCurrentPosition() <= RETRACTED_POSITION) {
            setState(State.IN);
        }
    }
    private void updateExtended() {
    }
    private void updateRetracted() {
    }
}
