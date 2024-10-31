package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Grabber extends LiftSubsystem {
    private final ServoImplEx grabServo;

    public Grabber(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);
        grabServo = hwMap.get(ServoImplEx.class, "LiftGrabServo");
        setPrepStatePositions(1.0, 0.0, 0.0, 0.0);
        setExecStatePositions(0.0, 1.0, 1.0, 1.0);
    }
    @Override
    public boolean executeCurrentState() {
        double pos = getExecStatePositions().get(getCurState());
        grabServo.setPosition(pos);
        return grabServo.getPosition() == pos;
    }

    public ServoImplEx getGrabServo() {
        return grabServo;
    }
}
