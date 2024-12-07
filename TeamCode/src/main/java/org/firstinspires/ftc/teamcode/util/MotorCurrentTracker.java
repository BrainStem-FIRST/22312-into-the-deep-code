package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorCurrentTracker {
    private final int MAX_NORMAL_CURRENT; // Note: this int MUST be in MilliAmps unit
    public final int ABNORMAL_VALIDATION_FRAMES;
    private int consecutiveAbnormalFrames;
    private final DcMotorEx motor;
    public MotorCurrentTracker(DcMotorEx motor, int MAX_NORMAL_CURRENT, int ABNORMAL_VALIDATION_FRAMES) {
        this.MAX_NORMAL_CURRENT = MAX_NORMAL_CURRENT;
        this.ABNORMAL_VALIDATION_FRAMES = ABNORMAL_VALIDATION_FRAMES;
        consecutiveAbnormalFrames = 0;
        this.motor = motor;
    }
    public boolean hasValidatedAbnormalCurrent() {
        return consecutiveAbnormalFrames >= ABNORMAL_VALIDATION_FRAMES;
    }
    public void updateCurrentTracking() {
        if(motor.getCurrent(CurrentUnit.MILLIAMPS) > MAX_NORMAL_CURRENT)
            consecutiveAbnormalFrames++;
        else
            consecutiveAbnormalFrames = 0;
    }

}
