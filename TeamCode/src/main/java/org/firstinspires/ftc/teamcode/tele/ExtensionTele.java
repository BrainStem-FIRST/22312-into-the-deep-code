package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class ExtensionTele extends Extension {

    final public static int EXTENDED_POSITION = 100;
    final public static int RETRACTED_POSITION = 0;
    final public static double EXTENDING_POWER = 0.2;
    final public static double RETRACTING_POWER = 0.2;
    final private PIDController pidController;

    public ExtensionTele(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);

        // TODO: FINE TUNE PID
        pidController = new PIDController(0.5, 0, 0);
    }
    public PIDController getPidController() {
        return pidController;
    }
}
