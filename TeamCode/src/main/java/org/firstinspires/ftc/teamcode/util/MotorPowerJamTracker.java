package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorPowerJamTracker {

    private final DcMotorEx motor;
    private double activationPowerRequirement;
    private final int jamEncoderTickRequirement;
    private final int jamFrameRequirement;

    // previous encoder position of motor
    private int lastEncoderTick;
    // number of frames collector has been jammed for
    private int jammedFrames;
    public MotorPowerJamTracker(DcMotorEx motor, double activationPowerRequirement, int jamEncoderTickRequirement, int jamFrameRequirement) {
        this.motor = motor;
        this.activationPowerRequirement = activationPowerRequirement;
        this.jamEncoderTickRequirement = jamEncoderTickRequirement;
        this.jamFrameRequirement = jamFrameRequirement;
        lastEncoderTick = motor.getCurrentPosition();
    }
    public void resetTracking() {
        jammedFrames = 0;
        lastEncoderTick = motor.getCurrentPosition();
    }
    public void updateJamTracking() {
        int distance = Math.abs(motor.getCurrentPosition() - lastEncoderTick); // disregard direction
        // double motorCurrent = spindleMotor.getCurrent();
        if (distance < jamEncoderTickRequirement)
            jammedFrames++;
        else
            jammedFrames = 0;

        lastEncoderTick = motor.getCurrentPosition();
    }

    public boolean isJammed() {
        return motor.getPower() >= activationPowerRequirement && jammedFrames > jamFrameRequirement;
    }
}
