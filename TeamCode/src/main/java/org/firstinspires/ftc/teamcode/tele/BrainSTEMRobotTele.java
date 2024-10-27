package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;

public class BrainSTEMRobotTele {
    // Don't touch these
    public Telemetry telemetry;
    public OpMode opMode;
    private AllianceColor allianceColor;

    public DriveTrainTele driveTrain;
    public LiftTele lift;
    public ExtensionTele extension;
    public CollectorTele collector;
    public GrabberTele grabber;
    public BrainSTEMRobotTele(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, AllianceColor allianceColor) {

        this.telemetry = telemetry;
        this.opMode = opMode;
        this.allianceColor = allianceColor;

        driveTrain = new DriveTrainTele(hwMap, telemetry, allianceColor);
        //lift = new LiftTele(hwMap, telemetry, allianceColor);
        //extension = new ExtensionTele(hwMap, telemetry, allianceColor);
        collector = new CollectorTele(hwMap, telemetry, allianceColor);
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public void update() {
        collector.update();

        // checking if collector found color and is hinging up
        if (collector.getHingeState() == Collector.HingeState.HINGING_UP)
            extension.setState(Extension.State.RETRACTING);
        extension.update();
    }

    public void extendAndCollect() {
        // ensuring collector is ready for extension before extension
        if (collector.getCollectState() == Collector.CollectState.EMPTY && collector.getHingeState() == Collector.HingeState.UP) {
            extension.setState(Extension.State.EXTENDING);
            // collector automatically sets hingeState to DOWN when it finishes hinging
            collector.setHingeState(Collector.HingeState.HINGING_DOWN);
            // collector keeps collecting until block is in chamber, then it sets collectState to FULL_MAX
            collector.setCollectState(Collector.CollectState.COLLECTING);
        }
    }

    // Note: this assumes collector does not have block in it
    public void resetExtensionAndCollector() {
        collector.setHingeState(Collector.HingeState.HINGING_UP);
        collector.setCollectState(Collector.CollectState.EMPTY);
        extension.setState(Extension.State.RETRACTING);

    }
}
