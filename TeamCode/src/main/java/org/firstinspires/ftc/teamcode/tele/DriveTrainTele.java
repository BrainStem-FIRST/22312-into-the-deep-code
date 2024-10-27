package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.DriveTrain;

public class DriveTrainTele extends DriveTrain {

    private int tick;

    private double leftStickX, leftStickY, rightStickX;
    private static final double threshold = 0.1;

    public DriveTrainTele (HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);
    }
}
