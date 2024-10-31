package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class Arm extends LiftSubsystem {
    private final ServoImplEx armServo;

    public Arm(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);
        armServo = hwMap.get(ServoImplEx.class, "LiftArmServo");

        setPrepStatePositions(0.0, 0.25, 0.5, 0.75);
        setExecStatePositions(0.0, 0.25, 0.5, 0.75);
    }
    @Override
    public boolean executeCurrentState() {
        return true;
        // shouldn't need this; in no case should arm move during execution of state
    }

    public ServoImplEx getArmServo() {
        return armServo;
    }
}
