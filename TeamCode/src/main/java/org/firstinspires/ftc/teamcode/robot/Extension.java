package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class Extension extends Subsystem {

    // TODO: find extension encoder ticks for these 3
    public static final int THRESHOLD = 30;
    public static final int RETRACTED_POSITION = 0;
    public static final int EXTENDED_POSITION = 1000;

    private final DcMotorEx extensionMotor;

    public Extension(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);

        extensionMotor = hwMap.get(DcMotorEx.class, "ExtensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotorEx getExtensionMotor() {
        return extensionMotor;
    }
    public void setExtensionMotorPosition(int position) {
        setMotorPosition(extensionMotor, position);
    }
}