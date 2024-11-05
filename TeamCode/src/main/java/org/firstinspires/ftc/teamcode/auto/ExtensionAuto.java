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

    // TODO - find position for extension to go to (to pickup yellow)
    public static final int AUTO_THRESHOLD = 5;
    public static final int YELLOW_PICKUP_TICK = 1000;
    public ExtensionAuto(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor, BrainSTEMRobot robot) {
        super(hwMap, telemetry, allianceColor, robot);
    }

    public Action extendForYellow() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setExtensionMotorPosition(YELLOW_PICKUP_TICK);
                return Math.abs(getExtensionMotor().getCurrentPosition() - YELLOW_PICKUP_TICK) < AUTO_THRESHOLD;
            }
        };
    }
}
