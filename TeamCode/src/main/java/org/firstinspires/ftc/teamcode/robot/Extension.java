package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class Extension extends Subsystem {
    private DcMotorEx extensionMotor;
    private ServoImplEx hingeServo;

    private enum States {
        IN, LOW, MED, HIGH
    }
    private States state = States.LOW;

    public Extension(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
    }
}