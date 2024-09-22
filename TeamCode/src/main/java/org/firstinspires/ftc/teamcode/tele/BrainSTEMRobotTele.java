package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BrainSTEMRobotTele {
    // Don't touch these
    public Telemetry telemetry;
    public OpMode opMode;

    public DriveTrainTele driveTrain;
    public LiftTele lift;
    public CollectorTele collector;
    public GrabberTele grabber;
    public BrainSTEMRobotTele(HardwareMap hwMap, Telemetry telemetry, OpMode opMode) {

        this.telemetry = telemetry;
        this.opMode = opMode;

        driveTrain = new DriveTrainTele(hwMap, telemetry);
        lift = new LiftTele(hwMap, telemetry);
        collector = new CollectorTele(hwMap, telemetry);
    }

    public void update() {
    }
}
