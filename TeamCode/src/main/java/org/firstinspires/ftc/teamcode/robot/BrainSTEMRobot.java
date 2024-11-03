package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tele.CollectorTele;

public class BrainSTEMRobot {
    public Telemetry telemetry;
    public OpMode opMode;
    private final AllianceColor allianceColor;

    public BrainSTEMRobot(Telemetry telemetry, OpMode opMode, AllianceColor allianceColor) {

        this.telemetry = telemetry;
        this.opMode = opMode;
        this.allianceColor = allianceColor;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }
}
