package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class Arm extends LiftSubsystem {
    // TODO: find arm servo positions
    public static final int MIN_TICK = 0, MAX_TICK = 100;
    public static final double TROUGH_POSITION = 0, BACKDROP_POSITION = 0.25, PICKUP_POSITION = 0.25, BASKET_POSITION = 0.25, RAM_POSITION = 0.75;
    private final ServoImplEx armServo;

    public Arm(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor, 0.1);
        armServo = hwMap.get(ServoImplEx.class, "LiftArmServo");
        armServo.setPwmRange(new PwmControl.PwmRange(MIN_TICK, MAX_TICK));

        setPrepStatePositions(TROUGH_POSITION, BACKDROP_POSITION, PICKUP_POSITION, BASKET_POSITION, RAM_POSITION);
        setExecStatePositions(TROUGH_POSITION, BACKDROP_POSITION, PICKUP_POSITION, BASKET_POSITION, RAM_POSITION);
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
