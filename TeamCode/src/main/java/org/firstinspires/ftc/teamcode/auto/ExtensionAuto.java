package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public class ExtensionAuto extends Extension {

    public ExtensionAuto(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);
    }

    public Action extendAction(int targetPosition) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setExtensionMotorPosition(targetPosition);
                return Math.abs(getExtensionMotor().getCurrentPosition() - targetPosition) > Extension.RETRACTED_THRESHOLD;
            }
        };
    }
    public Action retractAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setExtensionMotorPower(Extension.RETRACT_POWER);
                if (!magnetResetSwitch.getState()) {
                    setExtensionMotorPower(0);
                    return false;
                }
                return true;
            }
        };
    }
}
