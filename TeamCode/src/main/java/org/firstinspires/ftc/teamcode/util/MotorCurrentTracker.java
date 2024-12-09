package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorCurrentTracker {
    private final int maxNormalCurrent; // Note: this int MUST be in MilliAmps unit
    private final int abnormalValidationFrames;
    private int consecutiveAbnormalFrames;
    private int abnormalSafetyFrames;
    private int safetyFramesTimer;
    private final DcMotorEx motor;
    public MotorCurrentTracker(DcMotorEx motor, int maxNormalCurrent, int abnormalValidationFrames, int abnormalSafetyFrames) {
        this.motor = motor;
        this.maxNormalCurrent = maxNormalCurrent;
        this.abnormalValidationFrames = abnormalValidationFrames;
        this.abnormalSafetyFrames = abnormalSafetyFrames;
        consecutiveAbnormalFrames = 0;
        safetyFramesTimer = 0;
    }
    public int getConsecutiveAbnormalFrames() {
        return consecutiveAbnormalFrames;
    }
    public boolean hasValidatedAbnormalCurrent() {
        return consecutiveAbnormalFrames >= abnormalValidationFrames;
    }
    public boolean hasRawAbnormalCurrent() {
        return motor.getCurrent(CurrentUnit.MILLIAMPS) > maxNormalCurrent;
    }
    public void updateCurrentTracking() {
        // only check if not in safety
        if (safetyFramesTimer < abnormalSafetyFrames)
            if(motor.getCurrent(CurrentUnit.MILLIAMPS) > maxNormalCurrent)
                consecutiveAbnormalFrames++;
            else
                consecutiveAbnormalFrames = 0;

        // update safety
        if (hasValidatedAbnormalCurrent()) {
            safetyFramesTimer++;

            if (safetyFramesTimer > abnormalSafetyFrames)
                safetyFramesTimer = 0;
        }
    }

}
