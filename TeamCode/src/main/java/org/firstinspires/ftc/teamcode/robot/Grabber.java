package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Grabber extends LiftSubsystem {
    //TODO: find open and close positions for grabber
    public static final int MIN_TICK = 0, MAX_TICK = 100;
    public static final double CLOSE_POSITION = 0, OPEN_POSITION = 1;
    private final ServoImplEx grabServo;
    private boolean hasBlock = false;

    public Grabber(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor, 0.05);
        grabServo = hwMap.get(ServoImplEx.class, "LiftGrabServo");
        grabServo.setPwmRange(new PwmControl.PwmRange(MIN_TICK, MAX_TICK));
        setPrepStatePositions(OPEN_POSITION, CLOSE_POSITION, OPEN_POSITION, CLOSE_POSITION, CLOSE_POSITION);
        setExecStatePositions(CLOSE_POSITION, OPEN_POSITION, CLOSE_POSITION, OPEN_POSITION, OPEN_POSITION);
    }
    @Override
    public boolean executeCurrentState() {
        double pos = getExecStatePositions().get(getCurState());
        grabServo.setPosition(pos);
        return grabServo.getPosition() == pos;
    }

    // getters
    public ServoImplEx getGrabServo() {
        return grabServo;
    }
    public boolean hasBlock() {
        return this.hasBlock;
    }
}
